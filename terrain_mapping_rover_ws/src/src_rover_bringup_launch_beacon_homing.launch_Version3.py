"""
BEACON HOMING LAUNCH - Autonomous navigation to Bluetooth beacon

This is a MINIMAL launch that provides autonomous navigation to a
Bluetooth beacon WITHOUT requiring SLAM or Nav2.

Features:
- ✅ Reactive obstacle avoidance using ToF depth camera
- ✅ Signal gradient tracking to find beacon direction
- ✅ No prebuilt map required
- ✅ No SLAM required
- ✅ No Nav2 required
- ✅ Low CPU/memory usage

Usage:
    # Basic launch
    ros2 launch rover_bringup beacon_homing.launch.py
    
    # With specific beacon UUID
    ros2 launch rover_bringup beacon_homing. launch.py beacon_uuid:="your-uuid"
    
    # Mock sensors for testing
    ros2 launch rover_bringup beacon_homing.launch.py use_mock: =true
    
Then start homing: 
    ros2 service call /beacon_homing/start std_srvs/srv/SetBool "{data: true}"

Author:  Rover Team
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
    LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # =========================================================================
    # PACKAGE DIRECTORIES
    # =========================================================================
    pkg_bringup = get_package_share_directory('rover_bringup')
    pkg_description = get_package_share_directory('rover_description')
    pkg_vex = get_package_share_directory('vex_serial')
    pkg_imu = get_package_share_directory('imu_driver')
    pkg_tof = get_package_share_directory('arducam_tof')
    pkg_fusion = get_package_share_directory('sensor_fusion')
    pkg_bluetooth = get_package_share_directory('bluetooth_nav')
    pkg_beacon_homing = get_package_share_directory('beacon_homing')
    pkg_health = get_package_share_directory('mission_health')
    
    # =========================================================================
    # CONFIGURATION FILES
    # =========================================================================
    vex_config = os.path.join(pkg_vex, 'config', 'vex_serial_params.yaml')
    imu_config = os.path.join(pkg_imu, 'config', 'mpu6050_params.yaml')
    tof_config = os.path.join(pkg_tof, 'config', 'arducam_tof_params. yaml')
    ekf_config = os.path. join(pkg_fusion, 'config', 'ekf_params.yaml')
    bluetooth_config = os.path.join(pkg_bluetooth, 'config', 'bluetooth_params.yaml')
    beacon_homing_config = os.path. join(pkg_beacon_homing, 'config', 'beacon_homing_params.yaml')
    health_config = os. path.join(pkg_health, 'config', 'mission_health_params.yaml')
    
    # =========================================================================
    # LAUNCH ARGUMENTS
    # =========================================================================
    use_mock = LaunchConfiguration('use_mock')
    use_sim_time = LaunchConfiguration('use_sim_time')
    beacon_uuid = LaunchConfiguration('beacon_uuid')
    arrival_distance = LaunchConfiguration('arrival_distance')
    max_linear_vel = LaunchConfiguration('max_linear_vel')
    max_angular_vel = LaunchConfiguration('max_angular_vel')
    obstacle_threshold = LaunchConfiguration('obstacle_threshold')
    enable_health = LaunchConfiguration('enable_health')
    
    return LaunchDescription([
        # === ARGUMENTS ===
        DeclareLaunchArgument(
            'use_mock',
            default_value='false',
            description='Use mock sensor nodes for testing'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'beacon_uuid',
            default_value='',
            description='Target beacon UUID (empty = any beacon)'
        ),
        DeclareLaunchArgument(
            'arrival_distance',
            default_value='0.5',
            description='Distance to consider arrived (meters)'
        ),
        DeclareLaunchArgument(
            'max_linear_vel',
            default_value='0.2',
            description='Maximum linear velocity (m/s)'
        ),
        DeclareLaunchArgument(
            'max_angular_vel',
            default_value='1.0',
            description='Maximum angular velocity (rad/s)'
        ),
        DeclareLaunchArgument(
            'obstacle_threshold',
            default_value='0.4',
            description='Distance to consider obstacle blocking (meters)'
        ),
        DeclareLaunchArgument(
            'enable_health',
            default_value='true',
            description='Enable mission health monitoring'
        ),
        
        # === STARTUP MESSAGE ===
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  🎯 AUTONOMOUS BEACON HOMING SYSTEM'),
        LogInfo(msg='='*70),
        LogInfo(msg=''),
        LogInfo(msg='  This system navigates to a Bluetooth beacon using: '),
        LogInfo(msg='    - Signal gradient descent (RSSI tracking)'),
        LogInfo(msg='    - Reactive obstacle avoidance (ToF depth)'),
        LogInfo(msg=''),
        LogInfo(msg='  NO prebuilt map required! '),
        LogInfo(msg='  NO SLAM required!'),
        LogInfo(msg='  NO Nav2 required!'),
        LogInfo(msg=''),
        LogInfo(msg='='*70 + '\n'),
        
        # =====================================================================
        # PHASE 1: MINIMAL SENSORS (t=0s)
        # =====================================================================
        GroupAction([
            LogInfo(msg='>>> Starting minimal sensors (VEX, IMU, ToF)...\n'),
            
            # VEX Serial (motors + wheel odometry)
            Node(
                package='vex_serial',
                executable='vex_serial_node.py',
                name='vex_serial_node',
                output='screen',
                parameters=[vex_config, {'use_sim_time': use_sim_time}],
                condition=UnlessCondition(use_mock)
            ),
            Node(
                package='vex_serial',
                executable='vex_mock_node.py',
                name='vex_serial_node',
                output='screen',
                parameters=[vex_config, {'use_sim_time': use_sim_time}],
                condition=IfCondition(use_mock)
            ),
            
            # IMU
            Node(
                package='imu_driver',
                executable='mpu6050_node.py',
                name='mpu6050_node',
                output='screen',
                parameters=[imu_config, {'use_sim_time': use_sim_time}],
                condition=UnlessCondition(use_mock)
            ),
            Node(
                package='imu_driver',
                executable='imu_mock_node.py',
                name='mpu6050_node',
                output='screen',
                parameters=[imu_config, {'use_sim_time': use_sim_time}],
                condition=IfCondition(use_mock)
            ),
            
            # ToF Depth Camera (for obstacle avoidance)
            Node(
                package='arducam_tof',
                executable='arducam_tof_node.py',
                name='arducam_tof_node',
                output='screen',
                parameters=[tof_config, {'use_sim_time': use_sim_time}],
                condition=UnlessCondition(use_mock)
            ),
            Node(
                package='arducam_tof',
                executable='tof_mock_node.py',
                name='arducam_tof_node',
                output='screen',
                parameters=[tof_config, {'use_sim_time': use_sim_time}],
                condition=IfCondition(use_mock)
            ),
        ]),
        
        # =====================================================================
        # PHASE 2: ROBOT DESCRIPTION (t=0s)
        # =====================================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_description, 'launch', 'description.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        
        # =====================================================================
        # PHASE 3: EKF FUSION (t=2s)
        # =====================================================================
        TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg='>>> Starting EKF sensor fusion...\n'),
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[ekf_config, {'use_sim_time': use_sim_time}],
                    remappings=[('/odometry/filtered', '/odom')]
                ),
            ]
        ),
        
        # =====================================================================
        # PHASE 4: BLUETOOTH SCANNER (t=3s)
        # =====================================================================
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='>>> Starting Bluetooth beacon scanner...\n'),
                Node(
                    package='bluetooth_nav',
                    executable='beacon_scanner_node.py',
                    name='beacon_scanner_node',
                    output='screen',
                    parameters=[
                        bluetooth_config,
                        {'target_uuid': beacon_uuid}
                    ],
                ),
            ]
        ),
        
        # =====================================================================
        # PHASE 5: BEACON HOMING NODE (t=4s)
        # =====================================================================
        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg='>>> Starting Beacon Homing Node...\n'),
                Node(
                    package='beacon_homing',
                    executable='beacon_homing_node.py',
                    name='beacon_homing_node',
                    output='screen',
                    parameters=[
                        beacon_homing_config,
                        {'use_sim_time': use_sim_time},
                        {'target_uuid':  beacon_uuid},
                        {'arrival_distance':  arrival_distance},
                        {'max_linear_vel': max_linear_vel},
                        {'max_angular_vel': max_angular_vel},
                        {'obstacle_threshold':  obstacle_threshold},
                    ],
                ),
            ]
        ),
        
        # =====================================================================
        # PHASE 6: HEALTH MONITOR (t=5s) - Optional
        # =====================================================================
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='mission_health',
                    executable='mission_health_node.py',
                    name='mission_health_node',
                    output='screen',
                    parameters=[health_config, {'use_sim_time': use_sim_time}],
                    condition=IfCondition(enable_health)
                ),
            ]
        ),
        
        # =====================================================================
        # READY MESSAGE (t=6s)
        # =====================================================================
        TimerAction(
            period=6.0,
            actions=[
                LogInfo(msg='\n' + '='*70),
                LogInfo(msg='  ✅ BEACON HOMING SYSTEM READY! '),
                LogInfo(msg='='*70),
                LogInfo(msg=''),
                LogInfo(msg='  To START homing to beacon:'),
                LogInfo(msg='    ros2 service call /beacon_homing/start std_srvs/srv/SetBool "{data: true}"'),
                LogInfo(msg=''),
                LogInfo(msg='  To STOP homing:'),
                LogInfo(msg='    ros2 service call /beacon_homing/start std_srvs/srv/SetBool "{data: false}"'),
                LogInfo(msg=''),
                LogInfo(msg='  Monitor status:'),
                LogInfo(msg='    ros2 topic echo /beacon_homing/status'),
                LogInfo(msg=''),
                LogInfo(msg='  Detailed state:'),
                LogInfo(msg='    ros2 topic echo /beacon_homing/state'),
                LogInfo(msg=''),
                LogInfo(msg='  Health status:'),
                LogInfo(msg='    ros2 topic echo /mission_health'),
                LogInfo(msg=''),
                LogInfo(msg='='*70 + '\n'),
            ]
        ),
    ])