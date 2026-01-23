"""
Costmap Test Launch File

Launches only the costmap components for testing the terrain layer.

Usage:
    ros2 launch tmr_costmap costmap_test.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('tmr_costmap')
    
    # Config files
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'nav2_costmap.rviz')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true')
    
    # =========================================================================
    # Local Costmap Node (standalone for testing)
    # =========================================================================
    
    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[nav2_params],
        remappings=[
            ('odom', '/odometry/filtered'),
        ]
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
    # Costmap Diagnostics
    # =========================================================================
    
    costmap_diagnostics = Node(
        package='tmr_costmap',
        executable='costmap_diagnostics.py',
        name='costmap_diagnostics',
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
    # Info
    # =========================================================================
    
    startup_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR Costmap Test\n" +
            "="*60 + "\n" +
            "  Testing ToF Terrain Layer\n" +
            "  Ensure ToF camera is publishing to /tof/points\n" +
            "="*60
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        startup_msg,
        local_costmap_node,
        terrain_visualizer,
        costmap_diagnostics,
        rviz_node,
    ])