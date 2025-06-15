import rclpy
from rclpy.node import Node

from octomap_msgs.msg import Octomap
from moveit_msgs.msg import PlanningSceneWorld

class PlanningSceneWorldPublisher(Node):
    def __init__(self):
        super().__init__('planning_scene_world_publisher')

        self.publisher_ = self.create_publisher(
            PlanningSceneWorld,
            '/planning_scene_world',
            10
        )

        self.subscription = self.create_subscription(
            Octomap,
            '/octomap_full',
            self.octomap_callback,
            10
        )

        self.get_logger().info('Initialized: relaying /octomap_full to /planning_scene_world')

    def octomap_callback(self, msg: Octomap):
        scene_world = PlanningSceneWorld()
        scene_world.octomap.header = msg.header
        scene_world.octomap.octomap = msg

        self.publisher_.publish(scene_world)
        self.get_logger().info('Published PlanningSceneWorld with Octomap')

def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneWorldPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
