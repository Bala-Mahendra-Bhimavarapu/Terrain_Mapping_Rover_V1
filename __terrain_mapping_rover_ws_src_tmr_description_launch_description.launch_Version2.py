"""
Robot Description Launch File

Publishes robot URDF and all static transforms. 
Supports runtime parameter overrides for calibration. 

Usage:
    ros2 launch tmr_description description.launch.py
    
    # With calibration overrides: 
    ros2 launch tmr_description description.launch.py \
        camera_x:=0.14 camera_pitch:=-0.28
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('tmr_description')
    
    # URDF file path
    urdf_file = os.path.join(pkg_dir, 'urdf', 'rover.urdf.xacro')
    
    # =========================================================================
    # Launch Arguments for Calibration Overrides
    # =========================================================================
    
    # Camera position arguments
    camera_x_arg = DeclareLaunchArgument('camera_x', default_value='0.135')
    camera_y_arg = DeclareLaunchArgument('camera_y', default_value='-0.0275')
    camera_z_arg = DeclareLaunchArgument('camera_z', default_value='0.081')
    camera_roll_arg = DeclareLaunchArgument('camera_roll', default_value='0.0')
    camera_pitch_arg = DeclareLaunchArgument('camera_pitch', default_value='-0.2751')
    camera_yaw_arg = DeclareLaunchArgument('camera_yaw', default_value='0.0')
    
    # ToF position arguments
    tof_x_arg = DeclareLaunchArgument('tof_x', default_value='0.135')
    tof_y_arg = DeclareLaunchArgument('tof_y', default_value='0.014')
    tof_z_arg = DeclareLaunchArgument('tof_z', default_value='0.081')
    tof_roll_arg = DeclareLaunchArgument('tof_roll', default_value='0.0')
    tof_pitch_arg = DeclareLaunchArgument('tof_pitch', default_value='-0.2751')
    tof_yaw_arg = DeclareLaunchArgument('tof_yaw', default_value='0.0')
    
    # IMU position arguments
    imu_x_arg = DeclareLaunchArgument('imu_x', default_value='0.0')
    imu_y_arg = DeclareLaunchArgument('imu_y', default_value='0.0')
    imu_z_arg = DeclareLaunchArgument('imu_z', default_value='0.045')
    imu_roll_arg = DeclareLaunchArgument('imu_roll', default_value='0.0')
    imu_pitch_arg = DeclareLaunchArgument('imu_pitch', default_value='0.0')
    imu_yaw_arg = DeclareLaunchArgument('imu_yaw', default_value='0.0')
    
    # Process xacro with arguments
    robot_description_content = Command([
        'xacro ', urdf_file,
        ' camera_x:=', LaunchConfiguration('camera_x'),
        ' camera_y:=', LaunchConfiguration('camera_y'),
        ' camera_z:=', LaunchConfiguration('camera_z'),
        ' camera_roll: =', LaunchConfiguration('camera_roll'),
        ' camera_pitch:=', LaunchConfiguration('camera_pitch'),
        ' camera_yaw:=', LaunchConfiguration('camera_yaw'),
        ' tof_x:=', LaunchConfiguration('tof_x'),
        ' tof_y:=', LaunchConfiguration('tof_y'),
        ' tof_z:=', LaunchConfiguration('tof_z'),
        ' tof_roll:=', LaunchConfiguration('tof_roll'),
        ' tof_pitch:=', LaunchConfiguration('tof_pitch'),
        ' tof_yaw:=', LaunchConfiguration('tof_yaw'),
        ' imu_x:=', LaunchConfiguration('imu_x'),
        ' imu_y:=', LaunchConfiguration('imu_y'),
        ' imu_z: =', LaunchConfiguration('imu_z'),
        ' imu_roll:=', LaunchConfiguration('imu_roll'),
        ' imu_pitch:=', LaunchConfiguration('imu_pitch'),
        ' imu_yaw:=', LaunchConfiguration('imu_yaw'),
    ])
    
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # Robot state publisher - publishes TF from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {
            'publish_frequency': 50.0,
            'use_sim_time': False,
        }]
    )
    
    # Joint state publisher (publishes dummy joint states for visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'rate': 50,
        }]
    )
    
    return LaunchDescription([
        # Camera calibration arguments
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        camera_roll_arg,
        camera_pitch_arg,
        camera_yaw_arg,
        
        # ToF calibration arguments
        tof_x_arg,
        tof_y_arg,
        tof_z_arg,
        tof_roll_arg,
        tof_pitch_arg,
        tof_yaw_arg,
        
        # IMU calibration arguments
        imu_x_arg,
        imu_y_arg,
        imu_z_arg,
        imu_roll_arg,
        imu_pitch_arg,
        imu_yaw_arg,
        
        # Nodes
        robot_state_publisher,
        joint_state_publisher,
    ])