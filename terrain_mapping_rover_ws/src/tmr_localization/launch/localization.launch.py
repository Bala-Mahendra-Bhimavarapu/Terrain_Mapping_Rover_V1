"""
Full Localization Launch File

Launches EKF with diagnostics and monitoring tools. 

Usage:
    ros2 launch tmr_localization localization. launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('tmr_localization')
    description_dir = get_package_share_directory('tmr_description')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Enable EKF diagnostics node'
    )
    
    enable_comparison_arg = DeclareLaunchArgument(
        'enable_comparison',
        default_value='false',
        description='Enable raw vs filtered odometry comparison'
    )
    
    # =========================================================================
    # Robot Description (for TF tree)
    # =========================================================================
    
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'launch', 'description.launch.py')
        )
    )
    
    # =========================================================================
    # EKF Node
    # =========================================================================
    
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'ekf.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_tf': 'true',
        }.items()
    )
    
    # =========================================================================
    # Diagnostics Node
    # =========================================================================
    
    diagnostics_node = Node(
        package='tmr_localization',
        executable='ekf_diagnostics. py',
        name='ekf_diagnostics',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_diagnostics'))
    )
    
    # =========================================================================
    # Odometry Comparison Node
    # =========================================================================
    
    comparison_node = Node(
        package='tmr_localization',
        executable='odom_comparison.py',
        name='odom_comparison',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'raw_odom_topic': '/vex/odom_raw',
            'filtered_odom_topic': '/odometry/filtered',
        }],
        condition=IfCondition(LaunchConfiguration('enable_comparison'))
    )
    
    # =========================================================================
    # Info Message
    # =========================================================================
    
    info_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR Localization System\n" +
            "="*60 + "\n" +
            "  Components:\n" +
            "    - Robot State Publisher (TF)\n" +
            "    - EKF Sensor Fusion\n" +
            "    - Diagnostics (optional)\n" +
            "="*60
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        enable_diagnostics_arg,
        enable_comparison_arg,
        
        # Info
        info_msg,
        
        # Launch files
        robot_description_launch,
        ekf_launch,
        
        # Nodes
        diagnostics_node,
        comparison_node,
    ])
