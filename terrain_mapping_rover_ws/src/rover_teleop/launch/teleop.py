"""Launch file for teleop nodes."""

from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python. packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_teleop')
    default_config = os.path.join(pkg_dir, 'config', 'teleop_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('use_tank_mode', default_value='false',
                             description='Use tank mode (individual wheel control)'),
        
        # Standard keyboard teleop (Twist commands)
        Node(
            package='rover_teleop',
            executable='keyboard_teleop_node. py',
            name='keyboard_teleop_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            prefix='xterm -e',  # Run in separate terminal
        ),
    ])
