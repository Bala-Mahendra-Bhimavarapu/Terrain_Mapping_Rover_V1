"""
RTAB-MAP RGB-D SLAM Launch File

Specialized launch for RGB-D mode using IMX500 camera and Arducam ToF. 

Usage:
    ros2 launch tmr_slam slam_rgbd.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('tmr_slam')
    
    # Config
    config_file = os.path.join(pkg_dir, 'config', 'rtabmap_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_slam.rviz')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    
    low_texture_arg = DeclareLaunchArgument(
        'low_texture',
        default_value='false',
        description='Use low-texture optimized parameters'
    )
    
    # =========================================================================
    # RGB-D Sync Node
    # =========================================================================
    # Synchronizes RGB and Depth images
    
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.05,
            'queue_size': 10,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/tof/depth/image_raw'),
            ('rgbd_image', '/rgbd_image'),
        ]
    )
    
    # =========================================================================
    # RTAB-MAP SLAM Node
    # =========================================================================
    
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'subscribe_rgbd': True,
                'subscribe_rgb': False,
                'subscribe_depth': False,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'publish_tf': True,
                'approx_sync': True,
            }
        ],
        remappings=[
            ('rgbd_image', '/rgbd_image'),
            ('odom', '/odometry/filtered'),
            ('map', '/map'),
        ]
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
            "  TMR SLAM - RGB-D Mode\n" +
            "="*60 + "\n" +
            "  Using synchronized RGB-D images\n" +
            "  Camera: IMX500\n" +
            "  Depth:  Arducam ToF\n" +
            "="*60
    )
    
    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        low_texture_arg,
        startup_msg,
        rgbd_sync,
        rtabmap_node,
        rviz_node,
    ])