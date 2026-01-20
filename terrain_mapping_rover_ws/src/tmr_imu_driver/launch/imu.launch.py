"""
IMU Driver Launch File

Launches the MPU6050 IMU driver node. 

Usage:
    ros2 launch tmr_imu_driver imu.launch.py
    ros2 launch tmr_imu_driver imu. launch.py i2c_bus: =1 publish_rate: =100
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('tmr_imu_driver')
    
    # Declare launch arguments
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='1',
        description='I2C bus number'
    )
    
    i2c_address_arg = DeclareLaunchArgument(
        'i2c_address',
        default_value='0x68',
        description='I2C address of MPU6050'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='IMU publish rate in Hz'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU messages'
    )
    
    auto_calibrate_arg = DeclareLaunchArgument(
        'auto_calibrate',
        default_value='true',
        description='Auto-calibrate on startup'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'imu_params.yaml'),
        description='Path to configuration file'
    )
    
    # IMU node
    imu_node = Node(
        package='tmr_imu_driver',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'publish_rate_hz': LaunchConfiguration('publish_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'auto_calibrate': LaunchConfiguration('auto_calibrate'),
            }
        ],
        remappings=[
            ('imu/data', '/imu/data'),
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/temperature', '/imu/temperature'),
        ]
    )
    
    return LaunchDescription([
        i2c_bus_arg,
        i2c_address_arg,
        publish_rate_arg,
        frame_id_arg,
        auto_calibrate_arg,
        config_file_arg,
        imu_node,
    ])
