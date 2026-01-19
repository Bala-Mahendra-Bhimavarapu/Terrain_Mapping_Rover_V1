"""Launch file for mission health monitor."""

from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('mission_health')
    default_config = os.path.join(pkg_dir, 'config', 'mission_health_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        
        Node(
            package='mission_health',
            executable='mission_health_node. py',
            name='mission_health_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
