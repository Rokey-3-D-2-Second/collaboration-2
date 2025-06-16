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
            '/octomap_binary',
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

# import rclpy
# from rclpy.node import Node

# from sensor_msgs.msg import PointCloud2
# from octomap_msgs.msg import Octomap
# from moveit_msgs.msg import PlanningSceneWorld

# import tf2_ros
# import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs
# import sensor_msgs_py.point_cloud2 as pc2
# import numpy as np


# class PlanningSceneWorldPublisher(Node):
#     def __init__(self):
#         super().__init__('planning_scene_world_publisher')

#         # 구독: Octomap
#         self.create_subscription(
#             Octomap,
#             '/octomap_binary',
#             self.octomap_callback,
#             10
#         )

#         # 퍼블리셔: Octomap → PlanningSceneWorld
#         self.scene_pub = self.create_publisher(
#             PlanningSceneWorld,
#             '/planning_scene_world',
#             10
#         )

#         # 구독: Raw PointCloud2
#         self.create_subscription(
#             PointCloud2,
#             '/camera/camera/depth/color/points',
#             self.pointcloud_callback,
#             10
#         )

#         # 퍼블리셔: 필터링된 포인트클라우드
#         self.filtered_pub = self.create_publisher(
#             PointCloud2,
#             '/filtered_points',
#             10
#         )

#         # TF listener
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         self.get_logger().info('Initialized: Octomap relay + PointCloud filtering')

#     def octomap_callback(self, msg):
#         scene = PlanningSceneWorld()
#         scene.octomap.header = msg.header
#         scene.octomap.octomap = msg
#         self.scene_pub.publish(scene)

#     def pointcloud_callback(self, msg):
#         try:
#             # TF 변환: sensor → base_link
#             transform = self.tf_buffer.lookup_transform(
#                 'base_link',
#                 msg.header.frame_id,
#                 rclpy.time.Time()
#             )
#             pc_transformed = tf2_sensor_msgs.do_transform_cloud(msg, transform)

#             # PointCloud2 → numpy
#             points = pc2.read_points_numpy(pc_transformed, field_names=('x', 'y', 'z'), skip_nans=True)

#             # 필터링: base_link 기준 0.6m 이내 제거
#             distances = np.linalg.norm(points, axis=1)
#             filtered = points[distances > 0.6]

#             if filtered.shape[0] == 0:
#                 return

#             filtered_msg = pc2.create_cloud_xyz32(pc_transformed.header, filtered.tolist())
#             self.filtered_pub.publish(filtered_msg)

#         except Exception as e:
#             self.get_logger().warn(f'TF 또는 필터 오류: {e}')


# def main(args=None):
#     rclpy.init(args=args)
#     node = PlanningSceneWorldPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
