"""
EKF Launch File

Launches the robot_localization EKF node for sensor fusion. 
Fuses wheel odometry and IMU data. 

Usage:
    ros2 launch tmr_localization ekf.launch. py
    ros2 launch tmr_localization ekf. launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch. conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('tmr_localization')
    
    # Config file path
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether EKF should publish odom->base_link TF'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=ekf_config,
        description='Path to EKF configuration file'
    )
    
    output_odom_topic_arg = DeclareLaunchArgument(
        'output_odom_topic',
        default_value='/odometry/filtered',
        description='Topic name for filtered odometry output'
    )
    
    # =========================================================================
    # EKF Node (robot_localization)
    # =========================================================================
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            }
        ],
        remappings=[
            ('odometry/filtered', LaunchConfiguration('output_odom_topic')),
            # Remap input topics if needed
            # ('odom0', '/vex/odom_raw'),
            # ('imu0', '/imu/data'),
        ]
    )
    
    # =========================================================================
    # Info Message
    # =========================================================================
    
    info_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR EKF Sensor Fusion\n" +
            "="*60 + "\n" +
            "  Fusing:\n" +
            "    - Wheel odometry (/vex/odom_raw)\n" +
            "    - IMU data (/imu/data)\n" +
            "  Output:\n" +
            "    - Filtered odometry (/odometry/filtered)\n" +
            "    - TF:  odom -> base_link\n" +
            "="*60
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        publish_tf_arg,
        config_file_arg,
        output_odom_topic_arg,
        
        # Info
        info_msg,
        
        # Nodes
        ekf_node,
    ])