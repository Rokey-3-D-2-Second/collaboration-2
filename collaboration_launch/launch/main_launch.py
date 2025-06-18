from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        # 1. Realsense Launch
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                'initial_reset:=true',
                'enable_color:=true',
                'enable_depth:=true',
                'enable_sync:=true',
                'align_depth.enable:=true',
                'enable_rgbd:=true',
                'pointcloud.enable:=true',
                'decimation_filter.enable:=true',
                'decimation_filter.filter_magnitude:=8',
                'spatial_filter.enable:=true',
                'spatial_filter.smooth_alpha:=0.5',
                'spatial_filter.smooth_delta:=20',
                'spatial_filter.holes_fill:=2',
                'temporal_filter.enable:=true',
                'temporal_filter.smooth_alpha:=0.7',
                'temporal_filter.smooth_delta:=50',
                'depth_module.depth_profile:=640x480x30',
                'rgb_camera.color_profile:=640x480x30',
            ],
            output='screen'
        ),

        # 2. Static TF from link_6 to camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.015', '0.065', '0', '0', '-1.57', '-1.57', 'link_6', 'camera_link'],
            output='screen'
        ),

        # 3. Octomap Server
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'octomap_server', 'tracking_octomap_server_node',
                '--ros-args',
                # '-r', '/cloud_in:=/camera/camera/depth/color/points',
                '-r', '/cloud_in:=/filtered_points',
                '-p', 'frame_id:=base_link',
                '-p', 'resolution:=0.01',
                '-p', 'point_cloud_min_x:=0.15',
                '-p', 'point_cloud_max_x:=1.0',
                '-p', 'point_cloud_min_y:=-1.0',
                '-p', 'point_cloud_max_y:=1.0',
                '-p', 'point_cloud_min_z:=-0.05',
                '-p', 'point_cloud_max_z:=1.0',
                '-p', 'track_changes:=true',
                '-p', 'filter_speckles:=true',
                '-p', 'compress_map:=true',
                '-p', 'latch:=true',
                '-p', 'sensor_model.max_range:=1.0'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'realsense_camera', 'moveit'],
            output='screen'
        ),

        # 4. DSR Moveit RViz (real 모드)
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'launch', 'dsr_bringup2', 'dsr_bringup2_rviz.launch.py',
        #         'mode:=real',
        #         'model:=m0609',
        #         'host:=192.168.1.100'
        #     ],
        #     output='screen'
        # ),
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'dsr_bringup2', 'dsr_bringup2_moveit.launch.py',
                'mode:=real',
                'model:=m0609',
                'host:=192.168.1.100'
            ],
            output='screen'
        ),

        # 5. image_processor
        Node(
            package='image_processor',
            executable='image_processor',
            name='image_processor',
            output='screen'
        ),

        # 6. ros2_controller
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros2_controller',
                    executable='controller',
                    name='ros2_controller',
                    output='screen'
                ),
            ]
        ),
        
        # 7. vui
        # Node(
        #     package='vui',
        #     executable='vui',
        #     name='vui',
        #     output='screen'
        # ),
        TimerAction(
            period=25.0,  # 25초 후 실행
            actions=[
                Node(
                    package='vui',
                    executable='vui',
                    name='vui',
                    output='screen'
                )
            ]
        ),
    ])
