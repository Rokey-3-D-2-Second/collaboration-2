import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler
import math
from dsr_msgs2.srv import Ikin


# 정확한 orientation 변환 함수 추가
from tf_transformations import euler_matrix, quaternion_from_matrix

class IKMoveClient(Node):
    def __init__(self):
        super().__init__('ik_move_client')

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK 서비스 대기중...')

        self.cli = self.create_client(Ikin, '/motion/ikin')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK 서비스 대기중...')

        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/dsr_moveit_controller/follow_joint_trajectory'
        )

        self.send_target_pose()

    def send_target_pose(self):
        # 목표 pose
        # ik_req = GetPositionIK.Request()
        # pose = PoseStamped()
        # pose.header.frame_id = "base_link"
        # # pose.pose.position.x = 251.9 / 1000.0  # m
        # # pose.pose.position.y = 7.27 / 1000.0
        # # pose.pose.position.z = 479.5 / 1000.0
        # pose.pose.position.x = 200 / 1000.0  # m
        # pose.pose.position.y = 200 / 1000.0
        # pose.pose.position.z = 200 / 1000.0

        # Orientation (RPY to Quaternion)
        # quat = quaternion_from_euler(
        #     math.radians(90), math.radians(180), math.radians(180)
        # )
        # pose.pose.orientation.x = quat[0]
        # pose.pose.orientation.y = quat[1]
        # pose.pose.orientation.z = quat[2]
        # pose.pose.orientation.w = quat[3]
        # self.get_logger().info(f"quats: {quat}")

        # def doosan_euler_deg_to_quaternion(rx, ry, rz):
        #     # Doosan posx 기준은 ZYX 순서 (deg 단위)
        #     m = euler_matrix(
        #         math.radians(rz),
        #         math.radians(ry),
        #         math.radians(rx),
        #         axes='rzyx'
        #     )
        #     q = quaternion_from_matrix(m)
        #     return q[0], q[1], q[2], q[3]  # x, y, z, w
        
        # # rx, ry, rz = 78.1361, -179.9904, 167.9998
        # rx, ry, rz = 90, 180, 90
        # qx, qy, qz, qw = doosan_euler_deg_to_quaternion(rx, ry, rz)
        # pose.pose.orientation.x = qx
        # pose.pose.orientation.y = qy
        # pose.pose.orientation.z = qz
        # pose.pose.orientation.w = qw

        # ik_req.ik_request.group_name = "manipulator"
        # ik_req.ik_request.ik_link_name = "tool0"
        # # ik_req.ik_request.ik_link_name = "link_6"
        # ik_req.ik_request.pose_stamped = pose
        # self.get_logger().info(f"Pose: {pose.pose.position}, Orientation: {pose.pose.orientation}")

        # 현재 seed 상태 지정 (중요!)
        # joint_state = JointState()
        # joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        # joint_state.position = [-0.75, -31.5, 100.75, 0.0, 110.78, 89.44]       # rad or deg?
        # joint_state.position = [math.radians(p) for p in joint_state.position]  # Convert to rad

        # ik_req.ik_request.robot_state.joint_state = joint_state
        # self.get_logger().info(f"JointState name: {joint_state.name}")
        # self.get_logger().info(f"JointState position: {joint_state.position}")

        # # IK 요청
        # future = self.ik_client.call_async(ik_req)
        # rclpy.spin_until_future_complete(self, future)
        # result = future.result()

        # if result.error_code.val != 1:
        #     self.get_logger().error(f'IK 계산 실패 = {result.error_code.val}')
        #     return

        # joint_state = result.solution.joint_state

        req = Ikin.Request()
        # req.pos = [251.9128, 7.2731, 479.5822, 78.1361, -179.9904, 167.9998]
        # req.pos = [143.47349548339844, 7.150089263916016, 479.7123718261719, 97.31204223632812, -179.94760131835938, -172.81277465820312]
        req.pos = [251.9128875732422, 7.273128509521484, 479.5822448730469, 78.13613891601562, -179.99046325683594, 167.9998321533203]
        req.sol_space = 2
        req.ref = 0

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"IK 성공! 결과 posj: {future.result().conv_posj}")
        else:
            self.get_logger().error("IK 실패!")
            return
        
        joint_state = JointState()
        joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        joint_state.position = [math.radians(float(i)) for i in future.result().conv_posj]
        self.get_logger().info(f"IK 성공: {joint_state.position}")

        # 목표 trajectory 작성
        trajectory_msg = FollowJointTrajectory.Goal()
        valid_joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        joint_map = dict(zip(joint_state.name, joint_state.position))

        trajectory_msg.trajectory.joint_names = valid_joints
        point = JointTrajectoryPoint()

        try:
            point.positions = [joint_map[jn] for jn in valid_joints]
        except KeyError as e:
            self.get_logger().error(f"IK 결과에 누락된 joint: {e}")
            return

        point.time_from_start.sec = 5
        trajectory_msg.trajectory.points.append(point)

        # Trajectory 전송
        self.trajectory_client.wait_for_server()
        send_future = self.trajectory_client.send_goal_async(trajectory_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected!')
            return

        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Trajectory 실행 완료.')

def main(args=None):
    rclpy.init(args=args)
    node = IKMoveClient()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
