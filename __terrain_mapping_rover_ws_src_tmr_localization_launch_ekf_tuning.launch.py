"""
EKF Tuning Launch File

Launches EKF with tuning and visualization tools. 
Use this when calibrating the EKF parameters.

Usage:
    ros2 launch tmr_localization ekf_tuning.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('tmr_localization')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # =========================================================================
    # EKF Launch
    # =========================================================================
    
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'ekf. launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    
    # =========================================================================
    # Diagnostics Node
    # =========================================================================
    
    diagnostics_node = Node(
        package='tmr_localization',
        executable='ekf_diagnostics.py',
        name='ekf_diagnostics',
        output='screen',
    )
    
    # =========================================================================
    # Comparison Node (raw vs filtered)
    # =========================================================================
    
    comparison_node = Node(
        package='tmr_localization',
        executable='odom_comparison.py',
        name='odom_comparison',
        output='screen',
        parameters=[{
            'raw_odom_topic': '/vex/odom_raw',
            'filtered_odom_topic': '/odometry/filtered',
        }]
    )
    
    # =========================================================================
    # Covariance Tuner
    # =========================================================================
    
    tuner_node = Node(
        package='tmr_localization',
        executable='covariance_tuner.py',
        name='covariance_tuner',
        output='screen',
        prefix='xterm -e',
    )
    
    # =========================================================================
    # rqt_plot for visualization
    # =========================================================================
    
    # Plot odometry positions
    rqt_plot_position = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_plot', 'rqt_plot',
             '/vex/odom_raw/pose/pose/position/x',
             '/odometry/filtered/pose/pose/position/x',
             '/vex/odom_raw/pose/pose/position/y',
             '/odometry/filtered/pose/pose/position/y'],
        output='screen'
    )
    
    # =========================================================================
    # Info Message
    # =========================================================================
    
    info_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  EKF TUNING MODE\n" +
            "="*60 + "\n" +
            "  Tools running:\n" +
            "    - EKF node\n" +
            "    - Diagnostics monitor\n" +
            "    - Raw vs Filtered comparison\n" +
            "    - Interactive covariance tuner\n" +
            "    - rqt_plot for visualization\n" +
            "="*60 + "\n" +
            "  Tips:\n" +
            "    1. Drive the robot and observe plots\n" +
            "    2. Adjust covariances in the tuner\n" +
            "    3. Save successful parameters to config\n" +
            "="*60
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        
        # Info
        info_msg,
        
        # Launch
        ekf_launch,
        
        # Nodes
        diagnostics_node,
        comparison_node,
        tuner_node,
        rqt_plot_position,
    ])