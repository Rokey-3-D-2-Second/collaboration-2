import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from cb_interfaces.srv import TargetCoord
from cb_interfaces.action import TaskSteps

from .mover import Mover
from .forcer import Forcer
from .gripper import Gripper
from .motion_planner import Motion_Planner

from util import config, exceptions
from util.task_logger import get_logger  # Logger singleton
from collections import deque
import threading

from pathlib import Path
import DR_init

DR_init.__dsr__id = config.ROBOT_ID
DR_init.__dsr__model = config.ROBOT_MODEL


class Controller:
    """Pick‑and‑place 로봇 제어 + TaskLogger 연동 (개선판)"""

    def __init__(self, node: Node):
        self.node = node
        
        self.target_queue: deque = deque()
        self._task_running = False
        self._task_cond = threading.Condition()
        self.step_queue: deque = deque()

        # 하드웨어 모듈
        self.motion_planner = Motion_Planner(self.node)
        self.mover = Mover()
        self.forcer = Forcer()
        self.gripper = Gripper()

        # Logger (start_task는 매 Action 실행 시 호출)
        self.logger = get_logger()

        # Service: target coordinate
        self.target_coord_srv = self.node.create_service(
            TargetCoord, 
            config.TARGET_COORD, 
            self.handle_target_coord
        )

        # Action: 단계별 작업 수행
        self.task_steps_action_server = ActionServer(
            self.node,
            TaskSteps,
            config.TASK_STEPS,
            execute_callback=self.handle_task_steps,  # goal_callback 생략 → 모두 수락
        )

        # self.timer = self.node.create_timer(0.1, self._task_executor)

        # step 이름 ↔ 함수 매핑
        self.task_step_funcs = {
            "move_home": self.step_move_home,
            "move_target": self.step_move_target,
            "move_tray": self.step_move_tray,
            "force": self.step_force,
            "close_grip": self.step_close_grip,
            "open_grip": self.step_open_grip,
            "move_conta": self.step_detect_conta,
        }

    # ────────────────────────── step별 함수 정의 ────────────────────────── #

    # move
    def step_move_home(self, target=None):
        self.step_motion_planner_home()

    def step_move_target(self, target):
        self.step_motion_planner_target(target)

    def step_move_tray(self, target=None):
        self.step_motion_planner_tray()

    def step_detect_conta(self):
        self.step_motion_planner_conta()

    # force
    def step_force(self, target=None):
        self.forcer.force_on()
        self.forcer.check_touch()
        self.forcer.force_off()
        self.mover.up_little(5)

    # gripper
    def step_close_grip(self, target=None):
        self.gripper.close_grip()
        status = self.gripper.is_hold()
        if status == config.HOLDING:
            self.mover.up_little(120)

    def step_open_grip(self, target=None):
        status = self.gripper.is_hold()
        if status == config.HOLDING:
            self.gripper.open_grip()
        elif status == config.CLOSE:
            self.gripper.open_grip()
            self.mover.down_little(15)

    # motion planning helpers
    def step_motion_planner_home(self):
        self.motion_planner.wait_move_done()
        self.motion_planner.motion_planner_home()
        self.motion_planner.wait_move_done()

    def step_motion_planner_target(self, target):
        self.motion_planner.wait_move_done()
        self.motion_planner.motion_planner_target(target)
        self.motion_planner.wait_move_done()

    def step_motion_planner_tray(self):
        self.motion_planner.wait_move_done()
        self.motion_planner.motion_planner_tray()
        self.motion_planner.wait_move_done()

    def step_motion_planner_conta(self):
        self.motion_planner.wait_move_done()
        self.motion_planner.motion_planner_conta()
        self.motion_planner.wait_move_done()

    def set_dependencies(
        self, 
        check_motion,
    ):
        self.check_motion = check_motion

    # ────────────────────────── 초기 위치 이동 ────────────────────────── #
    def init_start(self):
        self.mover.move_to_scan()
        self.step_move_home()

    def init_pose(self):
        self.step_move_home()
        self.gripper.close_grip()
        
    def handle_target_coord(self, request, response):
        try:
            self.node.get_logger().info(f"request: {request.pose}")
            self.target_queue.append(request.pose)
            response.succeed = True
        except Exception as e:
            self.node.get_logger().error(f"target_coord exception: {e}")
            response.succeed = False
        return response

    # ────────────────────────── Action 콜백 (실행) ────────────────────────── #
    def handle_task_steps(self, goal_handle):
        # with self._task_cond:
        #     if self._task_running:
        #         self.step_queue.append(goal_handle)

        #         while self._task_running:   # 이미 실행 중이면 끝날 때까지 대기
        #             self.node.get_logger().info("[Controller] 다른 태스크 실행 중, 대기합니다.")
        #             self._task_cond.wait()
                
        #         goal_handle = self.step_queue.popleft()
            
        #     self._task_running = True

        self.logger.start_task()

        # self.step_queue.append(goal_handle)
        # return TaskSteps.Result()

        steps = goal_handle.request.steps
        success, message = True, "Task completed"
        idx, step = -1, ""

        try:
            wait_time = 0.0
            while not self.target_queue and wait_time < 5.0:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                wait_time += 0.1

            if not self.target_queue:
                err_msg = f"[Controller] target coordinate queue empty after waiting {wait_time:.1f} seconds"
                self.node.get_logger().error(err_msg)
                raise exceptions.ROS2_CONTROLLER_ERROR(303)

            target = self.target_queue.popleft()
            self.node.get_logger().info(f"[Controller] Got target coordinate: {target}")

            feedback = TaskSteps.Feedback()
            for idx, step in enumerate(steps):
                self.logger.log_step(step, "Start")

                feedback.current_step = idx
                feedback.step_desc = f"Executing: {step}"
                goal_handle.publish_feedback(feedback)

                func = self.task_step_funcs.get(step)
                if func is None:
                    err_msg = f"Unknown step '{step}' requested"
                    self.node.get_logger().error(err_msg)
                    raise exceptions.ROS2_CONTROLLER_ERROR(300)

                try:
                    if step == "move_target":
                        func(target)
                    else:
                        func()
                except:
                    raise exceptions.ROS2_CONTROLLER_ERROR(300)

            self.init_pose()

        except Exception as e:
            success = False
            message = f"Failed at step {idx + 1} ({step} : {e})"
            self.node.get_logger().error(message)

        finally:
            status_txt = "Tray setup complete" if success else "Failed"
            try:
                self.logger.complete_task(status_txt)
                self.logger.make_report(n=5)  # 경로 인자 없이 호출
            except Exception as e:
                self.node.get_logger().warn(f"Report error: {e}")

            result = TaskSteps.Result()
            result.success = success
            result.message = message
            (goal_handle.succeed if success else goal_handle.abort)()

            # 태스크 종료 알림
            # with self._task_cond:
            #     self._task_running = False
            #     self._task_cond.notify_all()
            
            return result
        
    def _task_executor(self):
        if self._task_running or not self.step_queue or not self.target_queue:
            return
        
        self.node.get_logger().info("run _task_executor")
        
        self._task_running = True
        target = self.target_queue.popleft()
        goal_handle = self.step_queue.popleft()

        steps = goal_handle.request.steps
        for idx, step in enumerate(steps):
            self.logger.log_step(step, "Start")

            func = self.task_step_funcs.get(step)
            if step == "move_target":
                func(target)
            else:
                func()

        self.init_pose()

    # ────────────────────────── 소멸자 ────────────────────────── #
    def __del__(self):
        self.node.destroy_node()


# ────────────────────────── 의존성 주입 함수 ────────────────────────── #

def init_modules(
    controller: Controller,
    movel,
    movej,
    check_motion,
    check_force_condition,
    task_compliance_ctrl,
    release_compliance_ctrl,
    set_desired_force,
    release_force,
    get_current_posx,
    DR_AXIS_Z,
    DR_FC_MOD_REL,
):
    controller.set_dependencies(check_motion)
    controller.mover.set_dependencies(movel, movej, get_current_posx)
    controller.forcer.set_dependencies(
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,
        DR_AXIS_Z,
        DR_FC_MOD_REL,
    )


# ────────────────────────── main ────────────────────────── #

def main():
    rclpy.init()
    node = rclpy.create_node("ros2_controller", namespace=config.ROBOT_ID)
    DR_init.__dsr__node = node
    controller = Controller(node)

    # DSR API 런타임 import
    from DSR_ROBOT2 import (
        get_tcp,
        get_tool,
        set_tcp,
        set_tool,
        set_ref_coord,
        movel,
        movej,
        check_motion,
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,
        get_current_posx,
        DR_AXIS_Z,
        DR_FC_MOD_REL,
    )

    init_modules(
        controller,
        movel,
        movej,
        check_motion,
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,
        get_current_posx,
        DR_AXIS_Z,
        DR_FC_MOD_REL,
    )

    # TCP & Tool 설정
    set_tcp(config.ROBOT_TCP)
    set_tool(config.ROBOT_TOOL)

    # 설정 검증
    if (tcp := get_tcp()) != config.ROBOT_TCP or (tool := get_tool()) != config.ROBOT_TOOL:
        node.destroy_node()
        rclpy.shutdown()
        return

    # executor = MultiThreadedExecutor()
    # executor.add_node(node)

    try:
        controller.init_start()
        while rclpy.ok():
            rclpy.spin_once(node)
        # executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        del controller
        node.destroy_node()
        rclpy.shutdown()