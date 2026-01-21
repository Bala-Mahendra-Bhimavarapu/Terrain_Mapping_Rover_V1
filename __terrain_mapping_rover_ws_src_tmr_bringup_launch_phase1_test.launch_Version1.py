"""
Phase 1 Test Launch File

Launches the Phase 1 system with test nodes for validation.

Usage:
    ros2 launch tmr_bringup phase1_test.launch.py
    ros2 launch tmr_bringup phase1_test.launch.py test: =motors
    ros2 launch tmr_bringup phase1_test.launch.py test:=sensors
    ros2 launch tmr_bringup phase1_test.launch.py test:=integration
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
from launch_ros. actions import Node


def generate_launch_description():
    # Get package directory
    bringup_dir = get_package_share_directory('tmr_bringup')
    
    # Declare launch arguments
    test_arg = DeclareLaunchArgument(
        'test',
        default_value='system',
        description='Test type: system, motors, sensors, integration, all'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(bringup_dir, 'config', 'phase1_params.yaml'),
        description='Path to configuration file'
    )
    
    # Full bringup for testing
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'phase1_bringup.launch. py')
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
            'teleop': 'none',
        }.items()
    )
    
    # System check test
    system_check = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tmr_bringup', 'phase1_system_check. py'],
                output='screen',
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration('test'), "' in ['system', 'all']"
                    ])
                )
            )
        ]
    )
    
    # Motor test
    motor_test = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tmr_bringup', 'phase1_motor_test.py'],
                output='screen',
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration('test'), "' in ['motors', 'all']"
                    ])
                )
            )
        ]
    )
    
    # Sensor test
    sensor_test = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tmr_bringup', 'phase1_sensor_test.py'],
                output='screen',
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration('test'), "' in ['sensors', 'all']"
                    ])
                )
            )
        ]
    )
    
    # Integration test
    integration_test = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tmr_bringup', 'phase1_integration_test.py'],
                output='screen',
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration('test'), "' in ['integration', 'all']"
                    ])
                )
            )
        ]
    )
    
    # Startup message
    startup_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR Phase 1 - Test Mode\n" +
            "="*60 + "\n" +
            "  Running Phase 1 validation tests...\n" +
            "="*60
    )
    
    return LaunchDescription([
        test_arg,
        config_file_arg,
        startup_msg,
        bringup_launch,
        system_check,
        motor_test,
        sensor_test,
        integration_test,
    ])