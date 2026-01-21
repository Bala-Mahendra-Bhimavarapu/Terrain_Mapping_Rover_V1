"""
Phase 1 Teleop Launch File

Launches VEX serial and teleop for manual control testing. 

Usage:
    ros2 launch tmr_bringup phase1_teleop.launch.py
    ros2 launch tmr_bringup phase1_teleop. launch.py mode:=gamepad
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    bringup_dir = get_package_share_directory('tmr_bringup')
    
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='keyboard',
        description='Teleop mode: keyboard or gamepad'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(bringup_dir, 'config', 'phase1_params.yaml'),
        description='Path to configuration file'
    )
    
    max_linear_arg = DeclareLaunchArgument(
        'max_linear',
        default_value='0.2',
        description='Maximum linear velocity (m/s)'
    )
    
    max_angular_arg = DeclareLaunchArgument(
        'max_angular',
        default_value='0.3',
        description='Maximum angular velocity (rad/s)'
    )
    
    # VEX Serial Node
    vex_serial_node = Node(
        package='tmr_vex_serial',
        executable='vex_serial_node',
        name='vex_serial_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'max_linear_velocity_ms': LaunchConfiguration('max_linear'),
                'max_angular_velocity_rads': LaunchConfiguration('max_angular'),
            }
        ],
    )
    
    # Keyboard Teleop
    teleop_keyboard = Node(
        package='tmr_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'max_linear_velocity':  LaunchConfiguration('max_linear'),
                'max_angular_velocity': LaunchConfiguration('max_angular'),
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'keyboard'"])
        )
    )
    
    # Gamepad Teleop
    gamepad_group = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'gamepad'"])
        ),
        actions=[
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
            ),
            Node(
                package='tmr_teleop',
                executable='teleop_gamepad',
                name='teleop_gamepad',
                output='screen',
                parameters=[
                    LaunchConfiguration('config_file'),
                    {
                        'max_linear_velocity':  LaunchConfiguration('max_linear'),
                        'max_angular_velocity': LaunchConfiguration('max_angular'),
                    }
                ],
            )
        ]
    )
    
    # Velocity smoother (optional)
    velocity_smoother = Node(
        package='tmr_teleop',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel_raw',
            'output_topic':  '/cmd_vel',
            'max_linear_acceleration': 0.5,
            'max_angular_acceleration': 1.0,
        }]
    )
    
    # Startup message
    startup_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR Phase 1 - Teleop Mode\n" +
            "="*60 + "\n" +
            "  Starting teleop.. .\n" +
            "  Max Linear:   " + "0.2" + " m/s\n" +
            "  Max Angular: " + "0.3" + " rad/s\n" +
            "="*60 + "\n" +
            "  SAFETY:  Be ready to press SPACE for emergency stop!\n" +
            "="*60
    )
    
    return LaunchDescription([
        mode_arg,
        config_file_arg,
        max_linear_arg,
        max_angular_arg,
        startup_msg,
        vex_serial_node,
        teleop_keyboard,
        gamepad_group,
    ])