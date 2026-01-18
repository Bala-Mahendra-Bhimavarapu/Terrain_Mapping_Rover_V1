"""Launch file for IMU driver."""

from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch. substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_imu_driver = get_package_share_directory('imu_driver')
    default_config = os.path.join(pkg_imu_driver, 'config', 'mpu6050_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        
        # Real IMU node
        Node(
            package='imu_driver',
            executable='mpu6050_node. py',
            name='mpu6050_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            condition=UnlessCondition(LaunchConfiguration('use_mock'))
        ),
        
        # Mock IMU node
        Node(
            package='imu_driver',
            executable='imu_mock_node. py',
            name='mpu6050_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            condition=IfCondition(LaunchConfiguration('use_mock'))
        ),
    ])
