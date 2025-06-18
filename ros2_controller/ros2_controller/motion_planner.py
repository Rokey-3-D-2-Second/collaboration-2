import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, RobotState
from dsr_msgs2.srv import Ikin

from util import config, exceptions
import math, time

class Motion_Planner:
    # home = config.homej
    home = config.homex
    tray = config.trayx

    def __init__(self, node: Node):
        self.node = node
        self._motion_done = True

        self.current_joint_state = None
        self.node.create_subscription(
            JointState,
            config.JOINT_STATE,
            self._joint_state_callback,
            10
        )

        self.ik_cli = self.node.create_client(
            Ikin, 
            config.MOTION_IKIN
        )

        self.movegroup_client = ActionClient(
            self.node,
            MoveGroup,
            config.MOVE_ACTION
        )

        self.execute_trajectory_client = ActionClient(
            self.node,
            ExecuteTrajectory,
            config.EXECUTE_TRAJECTORY
        )

    def _joint_state_callback(self, msg):
        self.current_joint_state = msg

    def motion_planner_home(self):
        self._motion_done = False
        # self._movegroup_planning(self.home)
        self._ik(self.home)
    
    def motion_planner_tray(self):
        self._motion_done = False
        self._ik(self.tray)

    def motion_planner_target(self, target):
        self._motion_done = False
        target[2] += 15
        self._ik([float(i) for i in target])

    def _ik(self, posx):
        req = Ikin.Request()
        req.pos = posx
        req.sol_space = 2
        req.ref = 0

        future = self.ik_cli.call_async(req)
        future.add_done_callback(self._ik_callback)

    def _ik_callback(self, future):
        if future.result().success:
            self.node.get_logger().info(f"IK 성공! 결과 posj: {future.result().conv_posj}")
            # IK 결과를 MoveGroup 액션으로 넘김
            self._movegroup_planning(future.result().conv_posj)
        else:
            self.node.get_logger().error("IK 실패!")
            self._motion_done = True

    def _movegroup_planning(self, posj):
        self.node.get_logger().info(f"[플래닝] MoveGroup 액션 goal 생성 (posj: {posj})")
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = "manipulator"
        goal_msg.request.allowed_planning_time = 5.0            # Planning Time (s)
        goal_msg.request.num_planning_attempts = 10             # Planning Attempts
        goal_msg.request.max_velocity_scaling_factor = 1.0      # Velocity Scaling
        goal_msg.request.max_acceleration_scaling_factor = 1.0  # Accel. Scaling
        # goal_msg.request.planner_id = config.PLANNER_IDS[0]
        goal_msg.request.planner_id = ""

        # goal_msg.request.start_state = RobotState()
        # goal_msg.request.start_state.joint_state = self.current_joint_state

        self.node.get_logger().info("[플래닝] JointConstraints로 목표 지정")
        joint_constraints = []
        for name, value in zip(config.JOINT_NAMES, [math.radians(float(i)) for i in posj]):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(Constraints(joint_constraints=joint_constraints))

        if not self.movegroup_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("[플래닝] MoveGroup 액션 서버를 찾을 수 없습니다.")
            self._motion_done = True
            return
        self.node.get_logger().info(f"[플래닝] 액션 서버 연결 확인 및 goal 전송")
        self.node.get_logger().info(f"[플래닝] 전달 posj(rad): {[math.radians(float(i)) for i in posj]}")

        send_future = self.movegroup_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._movegroup_goal_response_callback)

    def _movegroup_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('[플래닝] MoveGroup goal rejected!')
            self._motion_done = True
            return

        self.node.get_logger().info('[플래닝] MoveGroup goal accepted. 플래닝 결과 대기...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._movegroup_result_callback)

    def _movegroup_result_callback(self, future):
        result = future.result().result
        if not result or not result.planned_trajectory or not result.planned_trajectory.joint_trajectory.points:
            self.node.get_logger().error('[플래닝] MoveGroup 플래닝 실패!')
            self._motion_done = True
            return

        self.node.get_logger().info(f'[플래닝] 플래닝 성공! trajectory point 개수: {len(result.planned_trajectory.joint_trajectory.points)}')
        self.node.get_logger().info('[실행] MoveIt이 생성한 trajectory를 ExecuteTrajectory로 실행 시작')
        self._send_execute_trajectory(result.planned_trajectory)

    def _send_execute_trajectory(self, planned_trajectory):
        self.node.get_logger().info(f'[실행] ExecuteTrajectory 액션 goal 생성')
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = planned_trajectory

        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("[실행] ExecuteTrajectory 액션 서버를 찾을 수 없습니다.")
            self._motion_done = True
            return

        self.node.get_logger().info('[실행] ExecuteTrajectory 액션 서버 연결 확인 및 goal 전송')
        send_future = self.execute_trajectory_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._execute_trajectory_goal_response_callback)

    def _execute_trajectory_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('[실행] ExecuteTrajectory goal rejected!')
            self._motion_done = True
            return

        self.node.get_logger().info('[실행] ExecuteTrajectory goal accepted. 실행 결과 대기...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._execute_trajectory_result_callback)

    def _execute_trajectory_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f'[실행] Trajectory 실행 완료: {result}')
        self._motion_done = True

    def wait_move_done(self):
        self.node.get_logger().info("움직임 완료 대기 중...")
        while not self._motion_done:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.05)

        stabilization = 1.0
        self.node.get_logger().info(f"움직임 완료 확인됨, 안정화 {stabilization}초")
        time.sleep(stabilization)
