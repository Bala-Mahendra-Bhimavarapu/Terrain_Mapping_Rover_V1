"""
TF Test Launch File

Launches robot description with TF visualization tools for testing. 

Usage:
    ros2 launch tmr_description tf_test.launch.py
    ros2 launch tmr_description tf_test.launch.py rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('tmr_description')
    
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch joint_state_publisher_gui'
    )
    
    # Include robot description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'description.launch.py')
        )
    )
    
    # TF tree visualization (generates PDF)
    tf_tree_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_tools', 'view_frames'],
        output='screen',
        shell=False
    )
    
    # Static TF for odom frame (for testing without VEX)
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output='screen'
    )
    
    # Static TF for map frame (for testing without SLAM)
    static_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # RViz
    rviz_config = os.path.join(pkg_dir, 'config', 'tf_test.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # TF Monitor node
    tf_monitor = Node(
        package='tmr_description',
        executable='tf_monitor. py',
        name='tf_monitor',
        output='screen'
    )
    
    return LaunchDescription([
        rviz_arg,
        gui_arg,
        robot_description_launch,
        static_odom_tf,
        static_map_tf,
        rviz_node,
        tf_monitor,
    ])