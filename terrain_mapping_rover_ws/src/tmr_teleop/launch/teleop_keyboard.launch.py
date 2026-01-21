"""
Keyboard Teleop Launch File

Launches keyboard teleoperation node. 

Usage:
    ros2 launch tmr_teleop teleop_keyboard.launch.py
    ros2 launch tmr_teleop teleop_keyboard.launch. py max_linear: =0.5

Note:  This launch file starts the node, but you need to run it in a terminal
      where you can provide keyboard input.  Consider using: 
      ros2 run tmr_teleop teleop_keyboard
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('tmr_teleop')
    
    # Declare launch arguments
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
    
    # Keyboard teleop node
    # Note: For keyboard input to work, this needs to run in a terminal
    teleop_keyboard_node = Node(
        package='tmr_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',  # Opens in new terminal window
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
        ]
    )
    
    return LaunchDescription([
        max_linear_arg,
        max_angular_arg,
        cmd_vel_topic_arg,
        config_file_arg,
        teleop_keyboard_node,
    ])
