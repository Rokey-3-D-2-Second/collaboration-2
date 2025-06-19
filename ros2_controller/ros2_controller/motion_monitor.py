import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from control_msgs.msg import JointTrajectoryControllerState

class MotionMonitor(Node):
    def __init__(self):
        super().__init__('motion_monitor')
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/dsr_moveit_controller/controller_state',
            self.controller_state_callback,
            10
        )
        self.error_threshold = 0.05  # rad, 필요시 조정

    def controller_state_callback(self, msg):
        # 현재 각 조인트의 속도 출력
        velocities = msg.feedback.velocities
        joint_names = msg.joint_names
        vel_str = ', '.join([f"{name}: {vel:.3f} rad/s" for name, vel in zip(joint_names, velocities)])
        self.get_logger().info(f"현재 조인트 속도: {vel_str}")

def main(args=None):
    rclpy.init(args=args)
    node = MotionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()