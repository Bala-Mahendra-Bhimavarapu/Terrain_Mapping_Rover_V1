"""
Camera Driver Launch File

Launches the IMX500 camera driver node. 

Usage:
    ros2 launch tmr_camera camera.launch.py
    ros2 launch tmr_camera camera. launch.py width:=1920 height:=1080 frame_rate:=30
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('tmr_camera')
    
    # Declare launch arguments
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Camera index'
    )
    
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1280',
        description='Image width'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='720',
        description='Image height'
    )
    
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='30.0',
        description='Target frame rate'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_optical_frame',
        description='Frame ID for camera messages'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'camera_params.yaml'),
        description='Path to configuration file'
    )
    
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value=os.path. join(pkg_dir, 'config', 'imx500_calibration.yaml'),
        description='Path to calibration file'
    )
    
    # Camera node
    camera_node = Node(
        package='tmr_camera',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'camera_index': LaunchConfiguration('camera_index'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'calibration_file': LaunchConfiguration('calibration_file'),
            }
        ],
        remappings=[
            ('camera/image_raw', '/camera/image_raw'),
            ('camera/camera_info', '/camera/camera_info'),
            ('camera/image_raw/compressed', '/camera/image_raw/compressed'),
        ]
    )
    
    return LaunchDescription([
        camera_index_arg,
        width_arg,
        height_arg,
        frame_rate_arg,
        frame_id_arg,
        config_file_arg,
        calibration_file_arg,
        camera_node,
    ])
