import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from collections import deque

from cb_interfaces.srv import TargetCoord
from cb_interfaces.action import TaskSteps

from .mover import Mover
from .forcer import Forcer
from .gripper import Gripper

from util import config, exceptions

import DR_init
DR_init.__dsr__id = config.ROBOT_ID
DR_init.__dsr__model = config.ROBOT_MODEL

class Controller:
    def __init__(self, node: Node):
        self.node = node
        self.targets = deque()  # 여러 target 좌표를 큐로 관리

        # service server for target coord
        self.target_coord_srv = self.node.create_service(
            TargetCoord,
            config.TARGET_COORD,
            self.handle_target_coord
        )
        
        # action server for task_steps
        self.task_steps_action_server = ActionServer(
            self.node,
            TaskSteps,
            config.TASK_STEPS,
            self.handle_task_steps
        )

        self.mover = Mover()
        self.forcer = Forcer()
        self.gripper = Gripper()
        
        # step 이름과 함수 매핑
        self.task_step_funcs = {
            "move_home": self.step_move_home,
            "move_target": self.step_move_target,
            "move_tray": self.step_move_tray,
            "force": self.step_force,
            "close_grip": self.step_close_grip,
            "open_grip": self.step_open_grip,
        }

    # 각 step별 함수 정의
    def step_move_home(self, target=None):
        self.mover.move_to_home()

    def step_move_target(self, target):
        self.mover.move_to_target(target)
        
    def step_move_tray(self, target=None):
        self.mover.move_to_tray()

    def step_force(self, target=None):
        self.forcer.force_on()
        self.forcer.check_touch()
        self.forcer.force_off()
        self.mover.up_little()

    def step_close_grip(self, target=None):
        self.gripper.close_grip()
        self.mover.up_little()

    def step_open_grip(self, target=None):
        self.gripper.open_grip()
        self.mover.down_little()

    def set_dependencies(
        self, 
        check_motion,
    ):
        self.check_motion = check_motion
        
    def handle_target_coord(self, request, response):
        try:
            self.node.get_logger().info(f"request: {request.pose}")
            pose = request.pose
            self.targets.append(pose)

            response.succeed = True
            self.node.get_logger().info(f"resposne: {response.succeed}")
            return response

        except Exception as e:
            self.node.get_logger().error(f"target_coord service exception: {e}")
            response.succeed = False
            return response

    def handle_task_steps(self, goal_handle):
        steps = goal_handle.request.steps
        success = True
        message = "Task completed"

        try:
            self.mover.move_to_home()
            self.gripper.close_grip()

            # target 큐에서 하나 꺼내기 (없으면 5초 대기)
            wait_time = 0.0
            while not self.targets and wait_time < 5.0:
                self.node.get_logger().warn('Waiting for target coordinate...')
                rclpy.spin_once(self.node, timeout_sec=0.1)
                wait_time += 0.1
            
            if not self.targets:
                raise exceptions.ROS2_CONTROLLER_ERROR(303)
            
            target = self.targets.popleft()

            feedback = TaskSteps.Feedback()
            for idx, step in enumerate(steps):
                feedback.current_step = idx
                feedback.step_desc = f"Executing: {step}"
                goal_handle.publish_feedback(feedback)
                self.node.get_logger().info(f"피드백 publish: {feedback.current_step}, {feedback.step_desc}")

                func = self.task_step_funcs.get(step)
                if func is not None:
                    if step == "move_target":
                        func(target)
                    else:
                        func()
                else:
                    raise exceptions.ROS2_CONTROLLER_ERROR(300)

            self.mover.move_to_home()
            self.gripper.close_grip()

        except Exception as e:
            success = False
            message = f"Failed at step {idx+1} ({step} : {e})"
            self.node.get_logger().error(message)
        
        finally:
            result = TaskSteps.Result()
            result.success = success
            result.message = message
            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result

    def __del__(self):
        self.node.destroy_node()

def init_modules(
        controller: Controller,

        movel, 
        check_motion,
        
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,

        get_current_posx,

        DR_AXIS_Z,

        DR_FC_MOD_REL
    ):

    controller.set_dependencies(
        check_motion,
    )
    controller.mover.set_dependencies(
        movel, 

        get_current_posx,
    )
    controller.forcer.set_dependencies(
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,

        DR_AXIS_Z,

        DR_FC_MOD_REL
    )

def main():
    rclpy.init()
    node = rclpy.create_node('ros2_controller', namespace=config.ROBOT_ID)
    DR_init.__dsr__node = node
    controller = Controller(node)
    
    # import dsr api
    from DSR_ROBOT2 import (
        get_tcp, get_tool,
        set_tcp, set_tool, 
        set_ref_coord,

        movel, amovesx,
        check_motion,
        
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,

        get_current_posx,

        DR_AXIS_Z,

        DR_FC_MOD_REL
    )

    # init modules
    init_modules(  
        controller,

        movel, 
        check_motion,
        
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,

        get_current_posx,

        DR_AXIS_Z,

        DR_FC_MOD_REL
    )

    # set tcp & tool
    set_tcp(config.ROBOT_TCP)
    set_tool(config.ROBOT_TOOL)

    # check tcp & tool
    tcp, tool = get_tcp(), get_tool()
    print(f'tcp: {tcp}, tool: {tool}')
    if tcp != config.ROBOT_TCP or tool != config.ROBOT_TOOL:
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        print("Spin Start")
        while True:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        controller.forcer.force_off()
        node.get_logger().info("Shutting down ...")
    finally:
        del controller
        rclpy.shutdown()

if __name__ == 'main':
    main()