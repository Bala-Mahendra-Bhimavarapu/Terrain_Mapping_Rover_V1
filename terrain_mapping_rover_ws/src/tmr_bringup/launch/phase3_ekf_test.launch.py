"""
Phase 3 EKF Test Launch File

Launches EKF with testing and validation tools.

Usage:
    ros2 launch tmr_bringup phase3_ekf_test. launch.py
    ros2 launch tmr_bringup phase3_ekf_test.launch.py test:=fusion
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
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
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    test_arg = DeclareLaunchArgument(
        'test',
        default_value='all',
        description='Test type: all, ekf_check, fusion, odom, integration'
    )
    
    teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='keyboard',
        description='Teleop mode for integration testing'
    )
    
    # =========================================================================
    # Phase 3 Bringup
    # =========================================================================
    
    phase3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'phase3_bringup.launch.py')
        ),
        launch_arguments={
            'teleop': LaunchConfiguration('teleop'),
            'enable_diagnostics': 'false',  # We'll run our own
        }.items()
    )
    
    # =========================================================================
    # Test Nodes
    # =========================================================================
    
    # EKF check (verifies EKF is running and producing output)
    ekf_check = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tmr_bringup', 'phase3_ekf_check.py'],
                output='screen',
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration('test'), "' in ['all', 'ekf_check']"
                    ])
                )
            )
        ]
    )
    
    # Fusion test (verifies IMU + odom are being fused)
    fusion_test = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tmr_bringup', 'phase3_fusion_test.py'],
                output='screen',
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration('test'), "' in ['all', 'fusion']"
                    ])
                )
            )
        ]
    )
    
    # Odometry comparison test
    odom_test = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tmr_bringup', 'phase3_odom_test. py'],
                output='screen',
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration('test'), "' in ['all', 'odom']"
                    ])
                )
            )
        ]
    )
    
    # Full integration test
    integration_test = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tmr_bringup', 'phase3_integration_test.py'],
                output='screen',
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration('test'), "' in ['all', 'integration']"
                    ])
                )
            )
        ]
    )
    
    # =========================================================================
    # Startup Message
    # =========================================================================
    
    startup_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR Phase 3 - EKF Test Mode\n" +
            "="*60 + "\n" +
            "  Running EKF validation tests...\n" +
            "  ⚠️  Robot may move during integration test!\n" +
            "="*60
    )
    
    return LaunchDescription([
        test_arg,
        teleop_arg,
        startup_msg,
        phase3_bringup,
        ekf_check,
        fusion_test,
        odom_test,
        integration_test,
    ])
