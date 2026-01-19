"""
PHASE 3: EKF Sensor Fusion

This phase adds EKF odometry fusion:
- Fuses wheel odometry (/vex/odom_raw) with IMU (/imu/data)
- Outputs filtered odometry (/odom)
- Publishes odom → base_link transform

Prerequisites:
- Phase 1 (sensors) must be running
- Phase 2 (TF) must be running

Definition of Done:
✅ /odom topic publishing at ~50Hz
✅ TF:  odom → base_link available
✅ Odometry is smooth when driving (no jumps)
✅ Covariance values are reasonable

Test Commands:
    ros2 topic hz /odom
    ros2 topic echo /odom
    ros2 run tf2_ros tf2_echo odom base_link
    
Test with teleop:
    ros2 run rover_teleop keyboard_teleop_node. py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_bringup = get_package_share_directory('rover_bringup')
    pkg_fusion = get_package_share_directory('sensor_fusion')
    
    ekf_config = os.path. join(pkg_fusion, 'config', 'ekf_params.yaml')
    
    use_mock = LaunchConfiguration('use_mock')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # Startup message
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  PHASE 3: EKF SENSOR FUSION'),
        LogInfo(msg='='*70),
        LogInfo(msg='\n  Includes:  Phase 1 (Sensors) + Phase 2 (TF) + EKF'),
        LogInfo(msg='\n  Test with:'),
        LogInfo(msg='    ros2 topic hz /odom           (expect ~50Hz)'),
        LogInfo(msg='    ros2 topic echo /odom'),
        LogInfo(msg='    ros2 run tf2_ros tf2_echo odom base_link'),
        LogInfo(msg='\n  Test driving:'),
        LogInfo(msg='    ros2 run rover_teleop keyboard_teleop_node.py'),
        LogInfo(msg='\n' + '='*70 + '\n'),
        
        # Phase 1: Sensors
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'phase1_sensors.launch.py')
            ),
            launch_arguments={'use_mock': use_mock}.items()
        ),
        
        # Phase 2: TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os. path.join(pkg_bringup, 'launch', 'phase2_tf.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        
        # EKF (delay 2 seconds for sensors to start)
        TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg='\n>>> Starting EKF Filter Node.. .\n'),
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[ekf_config, {'use_sim_time': use_sim_time}],
                    remappings=[
                        ('/odometry/filtered', '/odom'),
                    ]
                ),
            ]
        ),
    ])