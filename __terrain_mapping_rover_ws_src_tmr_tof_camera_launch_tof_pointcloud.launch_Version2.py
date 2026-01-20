"""
ToF Point Cloud Launch File

Launches ToF camera with point cloud visualization in RViz.

Usage:
    ros2 launch tmr_tof_camera tof_pointcloud.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('tmr_tof_camera')
    
    # RViz argument
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    # Include base camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'tof_camera.launch.py')
        ),
        launch_arguments={
            'publish_pointcloud': 'true',
        }.items()
    )
    
    # Static transform publisher (if needed)
    # This publishes a transform from base_link to tof_optical_frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tof_static_tf',
        arguments=[
            '0.1', '0', '0.1',  # x, y, z translation
            '0', '0', '0',      # roll, pitch, yaw rotation
            'base_link',
            'tof_optical_frame'
        ]
    )
    
    return LaunchDescription([
        rviz_arg,
        camera_launch,
        static_tf_node,
    ])