"""
TF Test Launch File

Launches robot description with TF visualization tools for testing. 

Usage:
    ros2 launch tmr_description tf_test.launch.py
    ros2 launch tmr_description tf_test. launch.py rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch. substitutions import LaunchConfiguration, PythonExpression
from launch. conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('tmr_description')
    
    # Check for rviz config
    rviz_config = os.path. join(pkg_dir, 'config', 'tf_test. rviz')
    rviz_config_exists = os.path.exists(rviz_config)
    
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Launch joint_state_publisher_gui'
    )
    
    # Include robot description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os. path.join(pkg_dir, 'launch', 'description.launch.py')
        )
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
    
    # RViz - with or without config file
    if rviz_config_exists:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    else:
        # Launch without config, user can configure manually
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    
    # Info message
    info_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TF Test Mode\n" +
            "="*60 + "\n" +
            "  Robot description loaded\n" +
            "  Static odom->base_footprint published\n" +
            "  Static map->odom published\n" +
            f"  RViz config: {'Found' if rviz_config_exists else 'Not found (using defaults)'}\n" +
            "="*60 + "\n" +
            "  In RViz:\n" +
            "    1. Set Fixed Frame to 'odom'\n" +
            "    2. Add RobotModel display\n" +
            "    3. Add TF display\n" +
            "="*60
    )
    
    return LaunchDescription([
        rviz_arg,
        gui_arg,
        info_msg,
        robot_description_launch,
        static_odom_tf,
        static_map_tf,
        rviz_node,
    ])
