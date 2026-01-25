"""
Terrain Layer Test Launch File

Minimal launch for testing just the terrain layer classification.

Usage:
    ros2 launch tmr_costmap terrain_layer_test.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('tmr_costmap')
    
    rviz_config = os.path.join(pkg_dir, 'rviz', 'nav2_costmap.rviz')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    
    # =========================================================================
    # Test Terrain Layer Node
    # =========================================================================
    
    test_terrain_layer = Node(
        package='tmr_costmap',
        executable='test_terrain_layer.py',
        name='test_terrain_layer',
        output='screen'
    )
    
    # =========================================================================
    # Terrain Visualizer
    # =========================================================================
    
    terrain_visualizer = Node(
        package='tmr_costmap',
        executable='terrain_visualizer.py',
        name='terrain_visualizer',
        output='screen'
    )
    
    # =========================================================================
    # RViz
    # =========================================================================
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # =========================================================================
    # Run test script after delay
    # =========================================================================
    
    run_tests = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tmr_costmap', 'test_terrain_layer.py'],
                output='screen'
            )
        ]
    )
    
    # =========================================================================
    # Info
    # =========================================================================
    
    startup_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR Terrain Layer Test\n" +
            "="*60 + "\n" +
            "  Testing terrain classification from ToF point cloud\n" +
            "="*60
    )
    
    return LaunchDescription([
        rviz_arg,
        startup_msg,
        terrain_visualizer,
        rviz_node,
    ])
