"""
Phase 3 Complete Bringup Launch File

Launches all Phase 3 components (EKF Odometry Fusion):
- Phase 1 hardware (VEX, IMU, cameras)
- Phase 2 TF tree
- EKF sensor fusion
- Diagnostics

Usage:
    ros2 launch tmr_bringup phase3_bringup.launch.py
    ros2 launch tmr_bringup phase3_bringup.launch.py teleop:=keyboard
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
from launch.conditions import IfCondition, UnlessCondition
from launch_ros. actions import Node


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
        default_value='none',
        description='Teleop mode: none, keyboard, gamepad'
    )
    
    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Enable EKF diagnostics'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(bringup_dir, 'config', 'phase3_params.yaml'),
        description='Path to Phase 3 configuration file'
    )
    
    # =========================================================================
    # Phase 1 Hardware (VEX, IMU, Cameras)
    # =========================================================================
    
    phase1_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'phase1_bringup.launch.py')
        ),
        launch_arguments={
            'teleop': LaunchConfiguration('teleop'),
            'use_sim':  'false',
        }.items()
    )
    
    # =========================================================================
    # EKF Localization
    # =========================================================================
    
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_dir, 'launch', 'ekf. launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_tf': 'true',
        }.items()
    )
    
    # =========================================================================
    # EKF Diagnostics
    # =========================================================================
    
    ekf_diagnostics = Node(
        package='tmr_localization',
        executable='ekf_diagnostics.py',
        name='ekf_diagnostics',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_diagnostics'))
    )
    
    # =========================================================================
    # Startup Message
    # =========================================================================
    
    startup_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR Phase 3 Bringup - EKF Odometry Fusion\n" +
            "="*60 + "\n" +
            "  Components:\n" +
            "    - Phase 1: VEX Serial, IMU, Cameras\n" +
            "    - Phase 2: TF Tree (from URDF)\n" +
            "    - Phase 3: EKF Sensor Fusion\n" +
            "="*60 + "\n" +
            "  Sensor Fusion:\n" +
            "    - Wheel Odometry (/vex/odom_raw)\n" +
            "    - IMU (/imu/data)\n" +
            "    -> Filtered Odometry (/odometry/filtered)\n" +
            "="*60 + "\n" +
            "  TF Tree:\n" +
            "    odom -> base_link (from EKF)\n" +
            "="*60
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        teleop_arg,
        enable_diagnostics_arg,
        config_file_arg,
        
        # Startup message
        startup_msg,
        
        # Phase 1 hardware
        phase1_bringup,
        
        # EKF (delayed to allow sensors to start)
        TimerAction(
            period=3.0,
            actions=[ekf_launch]
        ),
        
        # Diagnostics (delayed further)
        TimerAction(
            period=5.0,
            actions=[ekf_diagnostics]
        ),
    ])
