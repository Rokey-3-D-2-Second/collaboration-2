import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from cb_interfaces.srv import Target, TargetCoord
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
        # super().__init__('drawer_and_eraser', namespace=ROBOT_ID)

        self.node = node

        # service server for target coord
        self.target_coord_srv = self.node.create_service(
            TargetCoord,
            '/target_coord',
            self.handle_target_coord
        )
        
        # action server for task_steps
        self.task_steps_action_server = ActionServer(
            self.node,
            TaskSteps,
            '/task_steps',
            self.handle_task_steps
        )

        self.mover = Mover()
        self.forcer = Forcer()
        self.gripper = Gripper()
        
        self.last_pose = None  # 최근 받은 좌표 저장용

    def set_dependencies(
        self, 
        check_motion,
    ):
        self.check_motion = check_motion
        
    def handle_target_coord(self, request, response):
        pose = request.pose  # [x, y, z, a, b, c]
        self.last_pose = pose  # 좌표 저장
        
        response.succeed = True
        return response

    def handle_task_steps(self, goal_handle):
        # steps: [move, force_on, force_off, close_grip, open_grip]
        steps = goal_handle.request.steps
        success = True
        message = "Task completed"

        try:
            # Feedback
            feedback = TaskSteps.Feedback()
            for idx, step in enumerate(steps):
                feedback.current_step = idx
                feedback.step_desc = f"Executing: {step}"
                goal_handle.publish_feedback(feedback)

                if step == "move":
                    # 최근 저장된 좌표가 있으면 그 좌표로 이동, 없으면 홈으로 이동
                    if self.last_pose is not None:
                        self.mover.move_to_target(self.last_pose)
                        self.last_pose = None
                    else:
                        self.mover.move_to_tray()
                elif step == "force":
                    self.forcer.force_on()
                    self.forcer.check_touch()
                    self.forcer.force_off()
                    self.mover.move_to_home()
                elif step == "close_grip":
                    if self.gripper.is_close():
                        self.gripper.open_grip()
                    self.gripper.close_grip()
                elif step == "open_grip":
                    if self.gripper.is_open():
                        self.gripper.close_grip()
                    self.gripper.open_grip()
                else:
                    raise exceptions.ROS2_CONTROLLER_ERROR(300)
        except Exception as e:
            success = False
            message = f"Failed at step {idx+1} ({step} : {e})"
            self.node.get_logger().error(message)
        finally:
            # Result
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
        pass

    del controller
    rclpy.shutdown()


if __name__ == 'main':
    main()