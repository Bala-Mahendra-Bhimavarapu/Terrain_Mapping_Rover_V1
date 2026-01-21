"""
Phase 1 Complete Bringup Launch File

Launches all Phase 1 components: 
- Robot description (URDF)
- VEX serial communication
- IMU driver
- Camera driver
- ToF camera driver
- Teleop (optional)

Usage:
    ros2 launch tmr_bringup phase1_bringup.launch.py
    ros2 launch tmr_bringup phase1_bringup.launch.py teleop: =keyboard
    ros2 launch tmr_bringup phase1_bringup.launch.py debug:=true
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
from launch. conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('tmr_bringup')
    description_dir = get_package_share_directory('tmr_description')
    
    # Declare launch arguments
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )
    
    teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='none',
        description='Teleop mode:  none, keyboard, gamepad'
    )
    
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation (skip hardware nodes)'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(bringup_dir, 'config', 'phase1_params.yaml'),
        description='Path to configuration file'
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
    # VEX Serial Node
    # =========================================================================
    vex_serial_node = Node(
        package='tmr_vex_serial',
        executable='vex_serial_node',
        name='vex_serial_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        condition=UnlessCondition(LaunchConfiguration('use_sim')),
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('odom', '/odom'),
            ('wheel_encoders', '/wheel_encoders'),
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
        parameters=[LaunchConfiguration('config_file')],
        condition=UnlessCondition(LaunchConfiguration('use_sim')),
        remappings=[
            ('imu/data', '/imu/data'),
            ('imu/data_raw', '/imu/data_raw'),
        ]
    )
    
    # =========================================================================
    # Camera Node (delayed start to allow other nodes to initialize)
    # =========================================================================
    camera_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='tmr_camera',
                executable='camera_node',
                name='camera_node',
                output='screen',
                parameters=[LaunchConfiguration('config_file')],
                condition=UnlessCondition(LaunchConfiguration('use_sim')),
                remappings=[
                    ('camera/image_raw', '/camera/image_raw'),
                    ('camera/camera_info', '/camera/camera_info'),
                ]
            )
        ]
    )
    
    # =========================================================================
    # ToF Camera Node (delayed start)
    # =========================================================================
    tof_camera_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='tmr_tof_camera',
                executable='tof_camera_node',
                name='tof_camera_node',
                output='screen',
                parameters=[LaunchConfiguration('config_file')],
                condition=UnlessCondition(LaunchConfiguration('use_sim')),
                remappings=[
                    ('tof/depth/image_raw', '/tof/depth/image_raw'),
                    ('tof/points', '/tof/points'),
                ]
            )
        ]
    )
    
    # =========================================================================
    # Teleop Nodes (conditional)
    # =========================================================================
    teleop_keyboard = Node(
        package='tmr_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[LaunchConfiguration('config_file')],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('teleop'), "' == 'keyboard'"])
        )
    )
    
    teleop_gamepad_group = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('teleop'), "' == 'gamepad'"])
        ),
        actions=[
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
                parameters=[{
                    'device_id': 0,
                    'deadzone': 0.1,
                }]
            ),
            Node(
                package='tmr_teleop',
                executable='teleop_gamepad',
                name='teleop_gamepad',
                output='screen',
                parameters=[LaunchConfiguration('config_file')],
            )
        ]
    )
    
    # =========================================================================
    # Startup message
    # =========================================================================
    startup_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR Phase 1 Bringup\n" +
            "="*60 + "\n" +
            "  Starting all Phase 1 components...\n" +
            "  - Robot Description\n" +
            "  - VEX Serial Communication\n" +
            "  - IMU Driver\n" +
            "  - RGB Camera\n" +
            "  - ToF Camera\n" +
            "="*60
    )
    
    return LaunchDescription([
        # Arguments
        debug_arg,
        teleop_arg,
        use_sim_arg,
        config_file_arg,
        
        # Startup message
        startup_msg,
        
        # Core nodes
        robot_description_launch,
        vex_serial_node,
        imu_node,
        camera_node,
        tof_camera_node,
        
        # Teleop (conditional)
        teleop_keyboard,
        teleop_gamepad_group,
    ])
