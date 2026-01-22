"""
Phase 2 TF Test Launch File

Launches TF tree with validation tests. 

Usage:
    ros2 launch tmr_bringup phase2_tf_test.launch.py
    ros2 launch tmr_bringup phase2_tf_test.launch.py test: =transforms
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
        description='Test type:  all, tf_check, transforms, validation'
    )
    
    # =========================================================================
    # Phase 2 Bringup
    # =========================================================================
    
    phase2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'phase2_bringup.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',
            'publish_static_odom': 'true',
        }.items()
    )
    
    # =========================================================================
    # Test Nodes
    # =========================================================================
    
    # TF tree check (runs after delay to allow TF to populate)
    tf_check = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tmr_bringup', 'phase2_tf_check. py'],
                output='screen',
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration('test'), "' in ['all', 'tf_check']"
                    ])
                )
            )
        ]
    )
    
    # Transform validation test
    transform_test = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tmr_bringup', 'phase2_transform_test.py'],
                output='screen',
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration('test'), "' in ['all', 'transforms']"
                    ])
                )
            )
        ]
    )
    
    # TF validator from tmr_description
    tf_validator = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tmr_description', 'tf_validator.py'],
                output='screen',
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration('test'), "' in ['all', 'validation']"
                    ])
                )
            )
        ]
    )
    
    # Generate TF tree PDF
    tf_tree_pdf = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tf2_tools', 'view_frames'],
                output='screen'
            )
        ]
    )
    
    # =========================================================================
    # Startup Message
    # =========================================================================
    
    startup_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR Phase 2 - TF Test Mode\n" +
            "="*60 + "\n" +
            "  Running TF validation tests...\n" +
            "  A frames. pdf will be generated showing the TF tree.\n" +
            "="*60
    )
    
    return LaunchDescription([
        test_arg,
        startup_msg,
        phase2_bringup,
        tf_check,
        transform_test,
        tf_validator,
        tf_tree_pdf,
    ])