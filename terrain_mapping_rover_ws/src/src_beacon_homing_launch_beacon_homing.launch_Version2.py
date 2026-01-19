"""
Beacon Homing Launch File

Launches ONLY what's needed for autonomous beacon navigation:
- Sensors (VEX, IMU, ToF)
- TF
- EKF
- Bluetooth Scanner
- Beacon Homing Node

NO SLAM, NO Nav2 - just reactive navigation to beacon! 

Usage:
    ros2 launch beacon_homing beacon_homing. launch.py
    
Then start homing: 
    ros2 service call /beacon_homing/start std_srvs/srv/SetBool "{data: true}"
"""

from launch import LaunchDescription
from launch. actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction,
    LogInfo,
    GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch. substitutions import LaunchConfiguration
from launch. launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python. packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    pkg_bringup = get_package_share_directory('rover_bringup')
    pkg_vex = get_package_share_directory('vex_serial')
    pkg_imu = get_package_share_directory('imu_driver')
    pkg_tof = get_package_share_directory('arducam_tof')
    pkg_description = get_package_share_directory('rover_description')
    pkg_fusion = get_package_share_directory('sensor_fusion')
    pkg_bluetooth = get_package_share_directory('bluetooth_nav')
    pkg_homing = get_package_share_directory('beacon_homing')
    
    # Configs
    vex_config = os.path. join(pkg_vex, 'config', 'vex_serial_params.yaml')
    imu_config = os.path.join(pkg_imu, 'config', 'mpu6050_params.yaml')
    tof_config = os.path.join(pkg_tof, 'config', 'arducam_tof_params. yaml')
    ekf_config = os.path. join(pkg_fusion, 'config', 'ekf_params.yaml')
    bluetooth_config = os.path.join(pkg_bluetooth, 'config', 'bluetooth_params.yaml')
    homing_config = os.path.join(pkg_homing, 'config', 'beacon_homing_params.yaml')
    
    # Launch arguments
    use_mock = LaunchConfiguration('use_mock')
    target_uuid = LaunchConfiguration('target_uuid')
    arrival_distance = LaunchConfiguration('arrival_distance')
    auto_start = LaunchConfiguration('auto_start')
    
    return LaunchDescription([
        # === ARGUMENTS ===
        DeclareLaunchArgument('use_mock', default_value='false',
            description='Use mock sensors'),
        DeclareLaunchArgument('target_uuid', default_value='',
            description='Target beacon UUID (empty = any beacon)'),
        DeclareLaunchArgument('arrival_distance', default_value='0.5',
            description='Distance to consider arrived (meters)'),
        DeclareLaunchArgument('auto_start', default_value='false',
            description='Auto-start homing on launch'),
        
        # === STARTUP MESSAGE ===
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  🎯 AUTONOMOUS BEACON HOMING SYSTEM'),
        LogInfo(msg='='*70),
        LogInfo(msg='\n  NO prebuilt map required!'),
        LogInfo(msg='  Uses reactive obstacle avoidance with ToF camera. '),
        LogInfo(msg='\n  Start homing: '),
        LogInfo(msg='    ros2 service call /beacon_homing/start std_srvs/srv/SetBool "{data: true}"'),
        LogInfo(msg='\n  Monitor status:'),
        LogInfo(msg='    ros2 topic echo /beacon_homing/status'),
        LogInfo(msg='\n' + '='*70 + '\n'),
        
        # === MINIMAL SENSORS ===
        GroupAction([
            # VEX Serial (motors + odometry)
            Node(
                package='vex_serial',
                executable='vex_serial_node. py',
                name='vex_serial_node',
                output='screen',
                parameters=[vex_config],
                condition=UnlessCondition(use_mock)
            ),
            Node(
                package='vex_serial',
                executable='vex_mock_node.py',
                name='vex_serial_node',
                output='screen',
                parameters=[vex_config],
                condition=IfCondition(use_mock)
            ),
            
            # IMU
            Node(
                package='imu_driver',
                executable='mpu6050_node.py',
                name='mpu6050_node',
                output='screen',
                parameters=[imu_config],
                condition=UnlessCondition(use_mock)
            ),
            Node(
                package='imu_driver',
                executable='imu_mock_node.py',
                name='mpu6050_node',
                output='screen',
                parameters=[imu_config],
                condition=IfCondition(use_mock)
            ),
            
            # ToF (for obstacle detection)
            Node(
                package='arducam_tof',
                executable='arducam_tof_node.py',
                name='arducam_tof_node',
                output='screen',
                parameters=[tof_config],
                condition=UnlessCondition(use_mock)
            ),
            Node(
                package='arducam_tof',
                executable='tof_mock_node.py',
                name='arducam_tof_node',
                output='screen',
                parameters=[tof_config],
                condition=IfCondition(use_mock)
            ),
        ]),
        
        # === ROBOT DESCRIPTION (TF) ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_description, 'launch', 'description.launch.py')
            )
        ),
        
        # === EKF (delayed 2s) ===
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
        
        # === BLUETOOTH SCANNER (delayed 3s) ===
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='bluetooth_nav',
                    executable='beacon_scanner_node. py',
                    name='beacon_scanner_node',
                    output='screen',
                    parameters=[bluetooth_config],
                ),
            ]
        ),
        
        # === BEACON HOMING NODE (delayed 4s) ===
        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg='\n>>> Starting Beacon Homing Node.. .\n'),
                Node(
                    package='beacon_homing',
                    executable='beacon_homing_node.py',
                    name='beacon_homing_node',
                    output='screen',
                    parameters=[
                        homing_config,
                        {'target_uuid': target_uuid},
                        {'arrival_distance': arrival_distance},
                    ],
                ),
            ]
        ),
        
        # === READY MESSAGE ===
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='\n' + '='*70),
                LogInfo(msg='  ✅ BEACON HOMING SYSTEM READY'),
                LogInfo(msg=''),
                LogInfo(msg='  Start homing with:'),
                LogInfo(msg='    ros2 service call /beacon_homing/start std_srvs/srv/SetBool "{data: true}"'),
                LogInfo(msg=''),
                LogInfo(msg='  Monitor with:'),
                LogInfo(msg='    ros2 topic echo /beacon_homing/status'),
                LogInfo(msg='='*70 + '\n'),
            ]
        ),
    ])