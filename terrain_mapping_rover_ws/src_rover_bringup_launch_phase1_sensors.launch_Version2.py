"""
PHASE 1: Sensor Bring-Up

This phase tests all hardware sensor drivers: 
- VEX Serial (wheel odometry + motor control)
- MPU6050 IMU
- IMX500 RGB Camera
- Arducam ToF Depth Camera

Definition of Done: 
✅ All sensors publishing data
✅ rostopic hz shows expected rates: 
   - /vex/odom_raw: 50Hz
   - /imu/data: 100Hz
   - /camera/image_raw: 30Hz
   - /tof/depth/image_raw: 15Hz
   - /tof/points: 15Hz
✅ rostopic echo shows valid data
✅ /vex/status shows connected=True

Test Commands:
    ros2 topic list
    ros2 topic hz /vex/odom_raw
    ros2 topic hz /imu/data
    ros2 topic hz /camera/image_raw
    ros2 topic hz /tof/depth/image_raw
    ros2 topic echo /vex/status
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch. substitutions import LaunchConfiguration
from launch_ros. actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    pkg_vex = get_package_share_directory('vex_serial')
    pkg_imu = get_package_share_directory('imu_driver')
    pkg_camera = get_package_share_directory('imx500_camera')
    pkg_tof = get_package_share_directory('arducam_tof')
    
    # Config files
    vex_config = os.path. join(pkg_vex, 'config', 'vex_serial_params.yaml')
    imu_config = os.path.join(pkg_imu, 'config', 'mpu6050_params.yaml')
    camera_config = os. path.join(pkg_camera, 'config', 'imx500_params.yaml')
    tof_config = os.path.join(pkg_tof, 'config', 'arducam_tof_params.yaml')
    
    # Launch arguments
    use_mock = LaunchConfiguration('use_mock')
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_mock',
            default_value='false',
            description='Use mock sensor nodes for testing without hardware'
        ),
        
        # Startup message
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  PHASE 1: SENSOR BRING-UP'),
        LogInfo(msg='='*70),
        LogInfo(msg='\n  Starting:  VEX Serial, IMU, Camera, ToF'),
        LogInfo(msg='  Use mock mode: ' + 'Checking launch arg... '),
        LogInfo(msg='\n  Test with: '),
        LogInfo(msg='    ros2 topic list'),
        LogInfo(msg='    ros2 topic hz /vex/odom_raw    (expect ~50Hz)'),
        LogInfo(msg='    ros2 topic hz /imu/data        (expect ~100Hz)'),
        LogInfo(msg='    ros2 topic hz /camera/image_raw (expect ~30Hz)'),
        LogInfo(msg='    ros2 topic hz /tof/points      (expect ~15Hz)'),
        LogInfo(msg='\n' + '='*70 + '\n'),
        
        # ===== VEX SERIAL =====
        # Real hardware
        Node(
            package='vex_serial',
            executable='vex_serial_node. py',
            name='vex_serial_node',
            output='screen',
            parameters=[vex_config],
            condition=UnlessCondition(use_mock)
        ),
        # Mock
        Node(
            package='vex_serial',
            executable='vex_mock_node.py',
            name='vex_serial_node',
            output='screen',
            parameters=[vex_config],
            condition=IfCondition(use_mock)
        ),
        
        # ===== IMU =====
        # Real hardware
        Node(
            package='imu_driver',
            executable='mpu6050_node.py',
            name='mpu6050_node',
            output='screen',
            parameters=[imu_config],
            condition=UnlessCondition(use_mock)
        ),
        # Mock
        Node(
            package='imu_driver',
            executable='imu_mock_node.py',
            name='mpu6050_node',
            output='screen',
            parameters=[imu_config],
            condition=IfCondition(use_mock)
        ),
        
        # ===== RGB CAMERA =====
        # Real hardware
        Node(
            package='imx500_camera',
            executable='imx500_camera_node.py',
            name='imx500_camera_node',
            output='screen',
            parameters=[camera_config],
            condition=UnlessCondition(use_mock)
        ),
        # Mock
        Node(
            package='imx500_camera',
            executable='camera_mock_node.py',
            name='imx500_camera_node',
            output='screen',
            parameters=[camera_config],
            condition=IfCondition(use_mock)
        ),
        
        # ===== TOF DEPTH CAMERA =====
        # Real hardware
        Node(
            package='arducam_tof',
            executable='arducam_tof_node.py',
            name='arducam_tof_node',
            output='screen',
            parameters=[tof_config],
            condition=UnlessCondition(use_mock)
        ),
        # Mock
        Node(
            package='arducam_tof',
            executable='tof_mock_node. py',
            name='arducam_tof_node',
            output='screen',
            parameters=[tof_config],
            condition=IfCondition(use_mock)
        ),
    ])