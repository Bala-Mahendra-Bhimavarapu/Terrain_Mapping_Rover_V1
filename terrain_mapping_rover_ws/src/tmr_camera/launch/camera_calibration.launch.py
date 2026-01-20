"""
Camera Calibration Launch File

Launches camera node and calibration tool for intrinsic calibration.

Usage:
    ros2 launch tmr_camera camera_calibration.launch.py
    
    Then show a checkerboard pattern to the camera. 
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch. substitutions import LaunchConfiguration
from launch_ros. actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('tmr_camera')
    
    # Checkerboard parameters
    size_arg = DeclareLaunchArgument(
        'size',
        default_value='8x6',
        description='Checkerboard size (inner corners)'
    )
    
    square_arg = DeclareLaunchArgument(
        'square',
        default_value='0.025',
        description='Checkerboard square size in meters'
    )
    
    # Include camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'camera.launch.py')
        )
    )
    
    # Calibration node
    calibration_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='cameracalibrator',
        output='screen',
        arguments=[
            '--size', LaunchConfiguration('size'),
            '--square', LaunchConfiguration('square'),
            '--no-service-check',
        ],
        remappings=[
            ('image', '/camera/image_raw'),
            ('camera', '/camera'),
        ]
    )
    
    return LaunchDescription([
        size_arg,
        square_arg,
        camera_launch,
        calibration_node,
    ])
