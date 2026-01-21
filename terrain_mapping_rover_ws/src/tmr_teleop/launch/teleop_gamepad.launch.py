"""
Gamepad Teleop Launch File

Launches joy node and gamepad teleoperation node. 

Usage:
    ros2 launch tmr_teleop teleop_gamepad.launch.py
    ros2 launch tmr_teleop teleop_gamepad.launch.py joy_dev:=/dev/input/js1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('tmr_teleop')
    
    # Declare launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )
    
    max_linear_arg = DeclareLaunchArgument(
        'max_linear',
        default_value='0.3',
        description='Maximum linear velocity (m/s)'
    )
    
    max_angular_arg = DeclareLaunchArgument(
        'max_angular',
        default_value='0.5',
        description='Maximum angular velocity (rad/s)'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Velocity command topic'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'teleop_params.yaml'),
        description='Path to configuration file'
    )
    
    # Joy node (reads from gamepad)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate':  20.0,
        }],
        remappings=[
            ('joy', '/joy'),
        ]
    )
    
    # Gamepad teleop node
    teleop_gamepad_node = Node(
        package='tmr_teleop',
        executable='teleop_gamepad',
        name='teleop_gamepad',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'max_linear_velocity': LaunchConfiguration('max_linear'),
                'max_angular_velocity': LaunchConfiguration('max_angular'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            }
        ],
        remappings=[
            ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
            ('joy', '/joy'),
        ]
    )
    
    return LaunchDescription([
        joy_dev_arg,
        max_linear_arg,
        max_angular_arg,
        cmd_vel_topic_arg,
        config_file_arg,
        joy_node,
        teleop_gamepad_node,
    ])
