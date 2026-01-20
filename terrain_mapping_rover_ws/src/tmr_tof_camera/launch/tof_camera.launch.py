"""
ToF Camera Driver Launch File

Launches the Arducam ToF camera driver node. 

Usage:
    ros2 launch tmr_tof_camera tof_camera.launch.py
    ros2 launch tmr_tof_camera tof_camera. launch.py depth_mode:=far frame_rate:=15
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('tmr_tof_camera')
    
    # Declare launch arguments
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='ToF camera device ID'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='tof_optical_frame',
        description='Frame ID for ToF sensor messages'
    )
    
    depth_mode_arg = DeclareLaunchArgument(
        'depth_mode',
        default_value='near',
        description='Depth mode:  near, middle, far'
    )
    
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='15.0',
        description='Target frame rate'
    )
    
    publish_pointcloud_arg = DeclareLaunchArgument(
        'publish_pointcloud',
        default_value='true',
        description='Whether to publish point cloud'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'tof_params.yaml'),
        description='Path to configuration file'
    )
    
    # ToF camera node
    tof_camera_node = Node(
        package='tmr_tof_camera',
        executable='tof_camera_node',
        name='tof_camera_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'device_id': LaunchConfiguration('device_id'),
                'frame_id': LaunchConfiguration('frame_id'),
                'depth_mode': LaunchConfiguration('depth_mode'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'publish_pointcloud': LaunchConfiguration('publish_pointcloud'),
            }
        ],
        remappings=[
            ('tof/depth/image_raw', '/tof/depth/image_raw'),
            ('tof/depth/camera_info', '/tof/depth/camera_info'),
            ('tof/confidence/image_raw', '/tof/confidence/image_raw'),
            ('tof/amplitude/image_raw', '/tof/amplitude/image_raw'),
            ('tof/points', '/tof/points'),
        ]
    )
    
    return LaunchDescription([
        device_id_arg,
        frame_id_arg,
        depth_mode_arg,
        frame_rate_arg,
        publish_pointcloud_arg,
        config_file_arg,
        tof_camera_node,
    ])
