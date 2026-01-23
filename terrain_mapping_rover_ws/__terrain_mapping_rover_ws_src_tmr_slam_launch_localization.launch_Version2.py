"""
RTAB-MAP Localization Only Launch File

Uses an existing map for localization without creating new map data. 
Useful for navigation in a previously mapped environment.

Usage:
    ros2 launch tmr_slam localization.launch.py database_path:=/path/to/map. db
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('tmr_slam')
    
    config_file = os.path.join(pkg_dir, 'config', 'rtabmap_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_slam.rviz')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        description='Path to RTAB-MAP database file (. db)'
    )
    
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    
    # =========================================================================
    # Remappings
    # =========================================================================
    
    remappings = [
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/tof/depth/image_raw'),
        ('odom', '/odometry/filtered'),
        ('map', '/map'),
    ]
    
    # =========================================================================
    # RTAB-MAP in Localization Mode
    # =========================================================================
    
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'database_path': LaunchConfiguration('database_path'),
                
                # LOCALIZATION MODE - key settings
                'Mem/IncrementalMemory': 'false',  # Don't add new nodes
                'Mem/InitWMWithAllNodes': 'true',  # Load all nodes to WM
                
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'publish_tf': True,
            }
        ],
        remappings=remappings,
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
            "  TMR SLAM - Localization Mode\n" +
            "="*60 + "\n" +
            "  Using existing map for localization only.\n" +
            "  No new map data will be created.\n" +
            "="*60
    )
    
    return LaunchDescription([
        database_path_arg,
        rviz_arg,
        use_sim_time_arg,
        startup_msg,
        rtabmap_localization,
        rviz_node,
    ])