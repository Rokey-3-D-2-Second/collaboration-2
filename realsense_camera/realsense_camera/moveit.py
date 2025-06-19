import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from octomap_msgs.msg import Octomap
from moveit_msgs.msg import PlanningSceneWorld

import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class PlanningSceneWorldPublisher(Node):
    def __init__(self):
        super().__init__('planning_scene_world_publisher')

        # 구독: Octomap
        self.create_subscription(
            Octomap,
            '/octomap_binary',
            self.octomap_callback,
            10
        )

        # 퍼블리셔: Octomap → PlanningSceneWorld
        self.scene_pub = self.create_publisher(
            PlanningSceneWorld,
            '/planning_scene_world',
            10
        )

        # 구독: Raw PointCloud2
        self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10
        )

        # 퍼블리셔: 필터링된 포인트클라우드
        self.filtered_pub = self.create_publisher(
            PointCloud2,
            '/filtered_points',
            10
        )

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Initialized: Octomap relay + PointCloud filtering')

    def octomap_callback(self, msg):
        scene = PlanningSceneWorld()
        scene.octomap.header = msg.header
        scene.octomap.octomap = msg
        self.scene_pub.publish(scene)

    def pointcloud_callback(self, msg):
        try:
            # # TF 변환: sensor → base_link
            # transform = self.tf_buffer.lookup_transform(
            #     'base_link',
            #     msg.header.frame_id,
            #     rclpy.time.Time()
            # )
            # pc_transformed = tf2_sensor_msgs.do_transform_cloud(msg, transform)

            # # PointCloud2 → numpy
            # points = pc2.read_points_numpy(pc_transformed, field_names=('x', 'y', 'z'), skip_nans=True)

            # # 필터링: base_link 기준 0.6m 이내 제거
            # distances = np.linalg.norm(points, axis=1)
            # filtered = points[distances > 0.5]

            # PointCloud2 → numpy (camera 좌표계 그대로 유지)
            points = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)

            # 카메라 좌표계 기준 전방 20cm 이내 제거 (z < 0.25m, y < 0.05m)
            # filtered = points[points[:, 1] < 0.025]
            # filtered = points[points[:, 2] > 0.25]

            # z > 0.2 AND y < 0.05 → 필터
            mask_y = points[:, 1] < 0.025
            mask_z = points[:, 2] > 0.275
            mask = np.logical_and(mask_y, mask_z)

            filtered = points[mask]  # 이 시점에 filtered.shape = (N, 3)

            if filtered.shape[0] == 0:
                return

            # filtered_msg = pc2.create_cloud_xyz32(pc_transformed.header, filtered.tolist())
            filtered_msg = pc2.create_cloud_xyz32(msg.header, filtered.tolist())
            self.filtered_pub.publish(filtered_msg)

        except Exception as e:
            self.get_logger().warn(f'TF 또는 필터 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneWorldPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
