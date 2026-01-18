"""
Launch file for VEX serial communication node.
"""

from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_vex_serial = get_package_share_directory('vex_serial')
    
    # Default config file
    default_config = os.path.join(pkg_vex_serial, 'config', 'vex_serial_params.yaml')
    
    # Declare arguments
    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Use mock node instead of real serial'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to parameter config file'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for VEX V5'
    )
    
    # Real VEX serial node
    vex_serial_node = Node(
        package='vex_serial',
        executable='vex_serial_node. py',
        name='vex_serial_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'serial_port': LaunchConfiguration('serial_port')}
        ],
        condition=launch. conditions.UnlessCondition(LaunchConfiguration('use_mock'))
    )
    
    # Mock node for testing
    vex_mock_node = Node(
        package='vex_serial',
        executable='vex_mock_node.py',
        name='vex_serial_node',  # Same name so topics match
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        condition=launch.conditions. IfCondition(LaunchConfiguration('use_mock'))
    )
    
    return LaunchDescription([
        use_mock_arg,
        config_file_arg,
        serial_port_arg,
        vex_serial_node,
        vex_mock_node,
    ])
