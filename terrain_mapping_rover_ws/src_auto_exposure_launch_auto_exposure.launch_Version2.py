"""Launch file for auto-exposure controller."""

from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python. packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('auto_exposure')
    default_config = os.path.join(pkg_dir, 'config', 'auto_exposure_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        
        Node(
            package='auto_exposure',
            executable='auto_exposure_node. py',
            name='auto_exposure_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])