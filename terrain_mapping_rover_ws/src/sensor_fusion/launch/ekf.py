"""Launch file for EKF sensor fusion."""

from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('sensor_fusion')
    default_config = os.path.join(pkg_dir, 'config', 'ekf_params. yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[
                ('/odometry/filtered', '/odom'),
            ]
        ),
    ])
