"""
Phase 1 Sensors Only Launch File

Launches only sensor nodes for testing:
- IMU driver
- Camera driver
- ToF camera driver

Usage:
    ros2 launch tmr_bringup phase1_sensors.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    bringup_dir = get_package_share_directory('tmr_bringup')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(bringup_dir, 'config', 'phase1_params.yaml'),
        description='Path to configuration file'
    )
    
    # IMU Node
    imu_node = Node(
        package='tmr_imu_driver',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )
    
    # Camera Node (delayed start)
    camera_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='tmr_camera',
                executable='camera_node',
                name='camera_node',
                output='screen',
                parameters=[LaunchConfiguration('config_file')],
            )
        ]
    )
    
    # ToF Camera Node (delayed start)
    tof_camera_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='tmr_tof_camera',
                executable='tof_camera_node',
                name='tof_camera_node',
                output='screen',
                parameters=[LaunchConfiguration('config_file')],
            )
        ]
    )
    
    # Startup message
    startup_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR Phase 1 - Sensors Only\n" +
            "="*60 + "\n" +
            "  Starting sensor nodes...\n" +
            "  - IMU (MPU6050)\n" +
            "  - RGB Camera (IMX500)\n" +
            "  - ToF Camera (Arducam B0410)\n" +
            "="*60
    )
    
    return LaunchDescription([
        config_file_arg,
        startup_msg,
        imu_node,
        camera_node,
        tof_camera_node,
    ])