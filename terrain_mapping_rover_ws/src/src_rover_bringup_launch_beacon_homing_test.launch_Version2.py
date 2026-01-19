"""
BEACON HOMING TEST - Test beacon homing with mock sensors

This launch file tests the beacon homing system using mock sensors,
including a simulated beacon at a configurable position. 

Perfect for testing without hardware! 

Usage:
    ros2 launch rover_bringup beacon_homing_test.launch.py
    
    # Then start homing: 
    ros2 service call /beacon_homing/start std_srvs/srv/SetBool "{data: true}"

Author: Rover Team
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
    LogInfo
)
from launch.substitutions import LaunchConfiguration
from launch. launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    pkg_bringup = get_package_share_directory('rover_bringup')
    pkg_description = get_package_share_directory('rover_description')
    pkg_vex = get_package_share_directory('vex_serial')
    pkg_imu = get_package_share_directory('imu_driver')
    pkg_tof = get_package_share_directory('arducam_tof')
    pkg_fusion = get_package_share_directory('sensor_fusion')
    pkg_bluetooth = get_package_share_directory('bluetooth_nav')
    pkg_beacon_homing = get_package_share_directory('beacon_homing')
    
    # Configs
    vex_config = os.path.join(pkg_vex, 'config', 'vex_serial_params.yaml')
    imu_config = os.path.join(pkg_imu, 'config', 'mpu6050_params.yaml')
    tof_config = os.path.join(pkg_tof, 'config', 'arducam_tof_params. yaml')
    ekf_config = os.path. join(pkg_fusion, 'config', 'ekf_params.yaml')
    beacon_homing_config = os.path. join(pkg_beacon_homing, 'config', 'beacon_homing_params.yaml')
    
    # Launch configurations
    beacon_x = LaunchConfiguration('beacon_x')
    beacon_y = LaunchConfiguration('beacon_y')
    
    return LaunchDescription([
        # === ARGUMENTS ===
        DeclareLaunchArgument(
            'beacon_x',
            default_value='3.0',
            description='X position of simulated beacon (meters)'
        ),
        DeclareLaunchArgument(
            'beacon_y',
            default_value='2.0',
            description='Y position of simulated beacon (meters)'
        ),
        
        # === STARTUP MESSAGE ===
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  🧪 BEACON HOMING TEST MODE'),
        LogInfo(msg='='*70),
        LogInfo(msg=''),
        LogInfo(msg='  Using MOCK sensors and simulated beacon'),
        LogInfo(msg='  Beacon position will be printed below'),
        LogInfo(msg=''),
        LogInfo(msg='='*70 + '\n'),
        
        # === MOCK SENSORS ===
        GroupAction([
            # Mock VEX
            Node(
                package='vex_serial',
                executable='vex_mock_node. py',
                name='vex_serial_node',
                output='screen',
                parameters=[vex_config],
            ),
            
            # Mock IMU
            Node(
                package='imu_driver',
                executable='imu_mock_node.py',
                name='mpu6050_node',
                output='screen',
                parameters=[imu_config],
            ),
            
            # Mock ToF
            Node(
                package='arducam_tof',
                executable='tof_mock_node.py',
                name='arducam_tof_node',
                output='screen',
                parameters=[tof_config],
            ),
        ]),
        
        # === ROBOT DESCRIPTION ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_description, 'launch', 'description.launch.py')
            )
        ),
        
        # === EKF (delay 2s) ===
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[ekf_config],
                    remappings=[('/odometry/filtered', '/odom')]
                ),
            ]
        ),
        
        # === MOCK BEACON SCANNER (delay 3s) ===
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='\n>>> Starting MOCK beacon scanner.. .\n'),
                Node(
                    package='bluetooth_nav',
                    executable='beacon_mock_node.py',
                    name='beacon_scanner_node',
                    output='screen',
                    parameters=[
                        {'beacon_x': beacon_x},
                        {'beacon_y':  beacon_y},
                        {'beacon_uuid': 'test-beacon-1234'},
                        {'noise_std': 3.0},
                        {'dropout_probability': 0.05},
                    ],
                ),
            ]
        ),
        
        # === BEACON HOMING NODE (delay 4s) ===
        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg='\n>>> Starting Beacon Homing Node...\n'),
                Node(
                    package='beacon_homing',
                    executable='beacon_homing_node.py',
                    name='beacon_homing_node',
                    output='screen',
                    parameters=[
                        beacon_homing_config,
                        {'target_uuid': ''},  # Accept any beacon in test mode
                        {'arrival_distance': 0.5},
                        {'max_linear_vel': 0.2},
                    ],
                ),
            ]
        ),
        
        # === READY MESSAGE (delay 5s) ===
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='\n' + '='*70),
                LogInfo(msg='  ✅ BEACON HOMING TEST READY! '),
                LogInfo(msg='='*70),
                LogInfo(msg=''),
                LogInfo(msg='  Simulated beacon at:  (' + str(beacon_x) + ', ' + str(beacon_y) + ')'),
                LogInfo(msg=''),
                LogInfo(msg='  Start homing: '),
                LogInfo(msg='    ros2 service call /beacon_homing/start std_srvs/srv/SetBool "{data: true}"'),
                LogInfo(msg=''),
                LogInfo(msg='  Monitor: '),
                LogInfo(msg='    ros2 topic echo /beacon_homing/status'),
                LogInfo(msg=''),
                LogInfo(msg='  Visualize:'),
                LogInfo(msg='    rviz2 -d $(ros2 pkg prefix rover_bringup)/share/rover_bringup/rviz/beacon_homing. rviz'),
                LogInfo(msg=''),
                LogInfo(msg='='*70 + '\n'),
            ]
        ),
    ])