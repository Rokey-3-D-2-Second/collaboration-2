import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from dsr_msgs2.srv import Ikin

from util import config, exceptions
import math, time

class Motion_Planner:
    home = config.homej

    def __init__(self, node: Node):
        self.node = node
        self._motion_done = True  # 초기값: 움직임 없음

        # self.node.create_subscription(
        #     JointState, 
        #     '/joint_states', 
        #     self.joint_states_callback, 
        #     10
        # )
        
        self.ik_cli = self.node.create_client(
            Ikin, 
            config.MOTION_IKIN
        )

        self.trajectory_client = ActionClient(
            self.node, 
            FollowJointTrajectory, 
            config.FOLLOW_JOINT_TRAJECTORY
        )

    def motion_planner_home(self):
        self._motion_done = False  # 움직임 시작됨 표시
        self._motion_planning(self.home)

    def motion_planner_target(self, target):
        self._motion_done = False  # 움직임 시작됨 표시
        target[2] += 15
        self._ik([float(i) for i in target])

    def motion_planner_tray(self):
        self._motion_done = False  # 움직임 시작됨 표시
        self._ik(config.trayx)
    
    # def joint_states_callback(self, msg: JointState):
    #     self.joint_state = msg.position

    def wait_move_done(self):
        self.node.get_logger().info("움직임 완료 대기 중...")
        while not self._motion_done:
            rclpy.spin_once(self.node, timeout_sec=0.1)  # ROS 이벤트 처리
            time.sleep(0.05)  # CPU 과점유 방지
        self.node.get_logger().info("움직임 완료 확인됨.")

    def _ik(self, posx):
        req = Ikin.Request()
        req.pos = posx
        req.sol_space = 2
        req.ref = 0

        future = self.ik_cli.call_async(req)
        future.add_done_callback(self._if_callback)
    
    def _if_callback(self, future):
        if future.result().success:
            self.node.get_logger().info(f"IK 성공! 결과 posj: {future.result().conv_posj}")
        else:
            self.node.get_logger().error("IK 실패!")
            return
        
        self._motion_planning(future.result().conv_posj)

    def _motion_planning(self, posj):
        joint_state = JointState()
        joint_state.name = config.JOINT_NAMES
        joint_state.position = [math.radians(float(i)) for i in posj]

        valid_joints = config.JOINT_NAMES
        joint_map = dict(zip(joint_state.name, joint_state.position))

        point = JointTrajectoryPoint()
        point.positions = [joint_map[jn] for jn in valid_joints]
        point.time_from_start.sec = 5

        trajectory_msg = FollowJointTrajectory.Goal()
        trajectory_msg.trajectory.joint_names = valid_joints
        trajectory_msg.trajectory.points.append(point)

        self._send_goal(trajectory_msg)

    def _send_goal(self, trajectory_msg):
        # 액션 서버 연결 확인
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("TaskSteps 액션 서버를 찾을 수 없습니다.")
            return
        
        self._motion_done = False  # 움직임 시작됨 표시
        send_future = self.trajectory_client.send_goal_async(trajectory_msg)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Trajectory goal rejected!')
            return

        self.node.get_logger().info('Goal accepted.')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f'Trajectory 실행 완료.: {result}')
        self._motion_done = True  # 움직임 완료 표시
