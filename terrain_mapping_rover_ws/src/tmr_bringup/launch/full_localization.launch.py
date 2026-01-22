"""
Full Localization Launch File

Launches complete system with all Phase 1-3 components. 
This is the main launch file for normal operation.

Usage:
    ros2 launch tmr_bringup full_localization.launch. py
    ros2 launch tmr_bringup full_localization.launch.py teleop:=gamepad
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('tmr_bringup')
    description_dir = get_package_share_directory('tmr_description')
    localization_dir = get_package_share_directory('tmr_localization')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='keyboard',
        description='Teleop mode: none, keyboard, gamepad'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug output'
    )
    
    # =========================================================================
    # Robot Description
    # =========================================================================
    
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'launch', 'description.launch.py')
        )
    )
    
    # =========================================================================
    # VEX Serial Node (with TF disabled - EKF will publish TF)
    # =========================================================================
    
    vex_serial_node = Node(
        package='tmr_vex_serial',
        executable='vex_serial_node',
        name='vex_serial_node',
        output='screen',
        parameters=[{
            'publish_tf': False,  # EKF will publish odom->base_link
        }],
        remappings=[
            ('odom', '/vex/odom_raw'),  # Rename to raw so EKF can use it
        ]
    )
    
    # =========================================================================
    # IMU Node
    # =========================================================================
    
    imu_node = Node(
        package='tmr_imu_driver',
        executable='imu_node',
        name='imu_node',
        output='screen',
    )
    
    # =========================================================================
    # Camera Nodes (delayed start)
    # =========================================================================
    
    camera_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='tmr_camera',
                executable='camera_node',
                name='camera_node',
                output='screen',
            )
        ]
    )
    
    tof_camera_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='tmr_tof_camera',
                executable='tof_camera_node',
                name='tof_camera_node',
                output='screen',
            )
        ]
    )
    
    # =========================================================================
    # EKF Localization
    # =========================================================================
    
    ekf_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os. path.join(localization_dir, 'launch', 'ekf.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'publish_tf': 'true',
                }.items()
            )
        ]
    )
    
    # =========================================================================
    # Teleop (conditional)
    # =========================================================================
    
    teleop_keyboard = Node(
        package='tmr_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('teleop'), "' == 'keyboard'"])
        )
    )
    
    teleop_gamepad = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('teleop'), "' == 'gamepad'"])
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
            )
        ]
    )
    
    # =========================================================================
    # RViz (optional)
    # =========================================================================
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # =========================================================================
    # Startup Message
    # =========================================================================
    
    startup_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR Full Localization System\n" +
            "="*60 + "\n" +
            "  Phase 1: Hardware Drivers\n" +
            "    - VEX Serial (/vex/odom_raw)\n" +
            "    - IMU (/imu/data)\n" +
            "    - Cameras\n" +
            "  Phase 2: TF Tree\n" +
            "    - Robot State Publisher\n" +
            "  Phase 3: Sensor Fusion\n" +
            "    - EKF (/odometry/filtered)\n" +
            "="*60 + "\n" +
            "  The robot is ready for teleoperation!\n" +
            "="*60
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        teleop_arg,
        rviz_arg,
        debug_arg,
        
        # Startup message
        startup_msg,
        
        # Robot description (TF tree)
        robot_description_launch,
        
        # Hardware drivers
        vex_serial_node,
        imu_node,
        camera_node,
        tof_camera_node,
        
        # EKF localization
        ekf_launch,
        
        # Teleop
        teleop_keyboard,
        teleop_gamepad,
        
        # Visualization
        rviz_node,
    ])
