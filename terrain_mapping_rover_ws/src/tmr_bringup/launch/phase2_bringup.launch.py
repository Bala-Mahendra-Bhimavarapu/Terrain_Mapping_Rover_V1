"""
Phase 2 Complete Bringup Launch File

Launches all Phase 2 components (TF Setup & Static Transforms):
- Robot description (URDF with TF tree)
- Robot state publisher
- Joint state publisher
- Static transforms for testing

Usage:
    ros2 launch tmr_bringup phase2_bringup.launch.py
    ros2 launch tmr_bringup phase2_bringup.launch.py rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch. conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('tmr_bringup')
    description_dir = get_package_share_directory('tmr_description')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Launch joint_state_publisher_gui'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    publish_static_odom_arg = DeclareLaunchArgument(
        'publish_static_odom',
        default_value='true',
        description='Publish static odom->base_footprint for testing without hardware'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(bringup_dir, 'config', 'phase2_params.yaml'),
        description='Path to Phase 2 configuration file'
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
    # Static Transform for Testing (odom -> base_footprint)
    # =========================================================================
    # Only published when testing without VEX hardware
    
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_static_odom'))
    )
    
    # Static map -> odom for testing without SLAM
    static_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_static_odom'))
    )
    
    # =========================================================================
    # TF Monitor Node
    # =========================================================================
    
    tf_monitor = Node(
        package='tmr_description',
        executable='tf_monitor. py',
        name='tf_monitor',
        output='screen',
    )
    
    # =========================================================================
    # RViz (optional)
    # =========================================================================
    
    rviz_config = os.path.join(description_dir, 'config', 'tf_test.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # =========================================================================
    # Startup Message
    # =========================================================================
    
    startup_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR Phase 2 Bringup - TF Setup & Static Transforms\n" +
            "="*60 + "\n" +
            "  Components:\n" +
            "    - Robot Description (URDF)\n" +
            "    - Robot State Publisher (TF tree)\n" +
            "    - Static Transforms (for testing)\n" +
            "    - TF Monitor\n" +
            "="*60 + "\n" +
            "  TF Tree:\n" +
            "    map -> odom -> base_footprint -> base_link\n" +
            "                                      ├── imu_link\n" +
            "                                      ├── camera_link -> camera_optical_frame\n" +
            "                                      ├── tof_link -> tof_optical_frame\n" +
            "                                      └── wheels.. .\n" +
            "="*60
    )
    
    return LaunchDescription([
        # Arguments
        rviz_arg,
        gui_arg,
        use_sim_time_arg,
        publish_static_odom_arg,
        config_file_arg,
        
        # Startup message
        startup_msg,
        
        # Core components
        robot_description_launch,
        static_odom_tf,
        static_map_tf,
        tf_monitor,
        
        # Visualization
        rviz_node,
    ])
