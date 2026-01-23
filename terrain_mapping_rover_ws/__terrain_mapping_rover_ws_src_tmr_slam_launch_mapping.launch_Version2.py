"""
RTAB-MAP Mapping Launch File

Full mapping mode with database persistence.

Usage:
    ros2 launch tmr_slam mapping. launch.py
    ros2 launch tmr_slam mapping.launch.py map_name:=my_map
"""

import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch. conditions import IfCondition
from launch_ros.actions import Node


def generate_database_path(context, *args, **kwargs):
    """Generate database path from map name."""
    map_name = LaunchConfiguration('map_name').perform(context)
    pkg_dir = get_package_share_directory('tmr_slam')
    
    if not map_name:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        map_name = f"tmr_map_{timestamp}"
    
    db_path = os.path.join(pkg_dir, 'maps', f'{map_name}.db')
    return [LogInfo(msg=f"Database path: {db_path}")]


def generate_launch_description():
    pkg_dir = get_package_share_directory('tmr_slam')
    
    config_file = os.path.join(pkg_dir, 'config', 'rtabmap_params.yaml')
    config_low_texture = os.path.join(pkg_dir, 'config', 'rtabmap_low_texture.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_slam.rviz')
    maps_dir = os.path.join(pkg_dir, 'maps')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='',
        description='Name for the map (empty = auto-generate with timestamp)'
    )
    
    low_texture_arg = DeclareLaunchArgument(
        'low_texture',
        default_value='false',
        description='Use low-texture optimized config'
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
    # RTAB-MAP Mapping Node
    # =========================================================================
    
    rtabmap_mapping = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time':  LaunchConfiguration('use_sim_time'),
                
                # Database will be auto-generated
                'database_path': '',  # Set via map_saver later
                'delete_db_on_start': True,
                
                # MAPPING MODE
                'Mem/IncrementalMemory': 'true',
                
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'publish_tf': True,
                
                # Mapping-specific
                'Rtabmap/DetectionRate': '2. 0',
                'grid_map':  True,
                'cloud_map': True,
            }
        ],
        remappings=remappings,
    )
    
    # =========================================================================
    # Map Saver Node (for saving maps on demand)
    # =========================================================================
    
    map_saver = Node(
        package='tmr_slam',
        executable='map_saver. py',
        name='map_saver',
        output='screen',
        parameters=[{
            'maps_directory': maps_dir,
            'default_map_name': LaunchConfiguration('map_name'),
        }]
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
    # SLAM Diagnostics
    # =========================================================================
    
    slam_diagnostics = Node(
        package='tmr_slam',
        executable='slam_diagnostics.py',
        name='slam_diagnostics',
        output='screen',
    )
    
    # =========================================================================
    # Info
    # =========================================================================
    
    startup_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR SLAM - Mapping Mode\n" +
            "="*60 + "\n" +
            "  Creating new map.\n" +
            "  Use 'ros2 service call /save_map' to save.\n" +
            "="*60
    )
    
    return LaunchDescription([
        map_name_arg,
        low_texture_arg,
        rviz_arg,
        use_sim_time_arg,
        startup_msg,
        rtabmap_mapping,
        map_saver,
        slam_diagnostics,
        rviz_node,
    ])