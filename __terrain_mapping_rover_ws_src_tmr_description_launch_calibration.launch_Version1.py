"""
Calibration Mode Launch File

Launches the robot description with calibration tools for adjusting sensor positions.

Usage:
    ros2 launch tmr_description calibration.launch.py
    
    # Override camera position: 
    ros2 launch tmr_description calibration.launch.py camera_x:=0.14
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('tmr_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'rover.urdf. xacro')
    
    # =========================================================================
    # Calibration Arguments (all sensor positions adjustable)
    # =========================================================================
    
    # Camera position arguments
    camera_x_arg = DeclareLaunchArgument('camera_x', default_value='0.13',
        description='Camera X position (forward)')
    camera_y_arg = DeclareLaunchArgument('camera_y', default_value='-0.0275',
        description='Camera Y position (right is negative)')
    camera_z_arg = DeclareLaunchArgument('camera_z', default_value='0.078',
        description='Camera Z position (up)')
    camera_roll_arg = DeclareLaunchArgument('camera_roll', default_value='0.0',
        description='Camera roll angle')
    camera_pitch_arg = DeclareLaunchArgument('camera_pitch', default_value='-0.2751',
        description='Camera pitch angle (tilted down is negative)')
    camera_yaw_arg = DeclareLaunchArgument('camera_yaw', default_value='0.0',
        description='Camera yaw angle')
    
    # ToF position arguments
    tof_x_arg = DeclareLaunchArgument('tof_x', default_value='0.13',
        description='ToF X position')
    tof_y_arg = DeclareLaunchArgument('tof_y', default_value='0.014',
        description='ToF Y position')
    tof_z_arg = DeclareLaunchArgument('tof_z', default_value='0.078',
        description='ToF Z position')
    tof_roll_arg = DeclareLaunchArgument('tof_roll', default_value='0.0',
        description='ToF roll angle')
    tof_pitch_arg = DeclareLaunchArgument('tof_pitch', default_value='-0.2751',
        description='ToF pitch angle')
    tof_yaw_arg = DeclareLaunchArgument('tof_yaw', default_value='0.0',
        description='ToF yaw angle')
    
    # IMU position arguments
    imu_x_arg = DeclareLaunchArgument('imu_x', default_value='0.0',
        description='IMU X position')
    imu_y_arg = DeclareLaunchArgument('imu_y', default_value='0.0',
        description='IMU Y position')
    imu_z_arg = DeclareLaunchArgument('imu_z', default_value='0.046',
        description='IMU Z position')
    imu_roll_arg = DeclareLaunchArgument('imu_roll', default_value='0.0',
        description='IMU roll angle')
    imu_pitch_arg = DeclareLaunchArgument('imu_pitch', default_value='0.0',
        description='IMU pitch angle')
    imu_yaw_arg = DeclareLaunchArgument('imu_yaw', default_value='0.0',
        description='IMU yaw angle')
    
    # =========================================================================
    # Process URDF with arguments
    # =========================================================================
    
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
    
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }
    
    # =========================================================================
    # Nodes
    # =========================================================================
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {
            'publish_frequency': 50.0,
        }]
    )
    
    # Joint state publisher GUI (for interactive adjustment)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # Static transforms for testing
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
    )
    
    static_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    # Calibration helper node
    calibration_helper = Node(
        package='tmr_description',
        executable='calibration_helper. py',
        name='calibration_helper',
        output='screen'
    )
    
    # =========================================================================
    # Info message
    # =========================================================================
    
    info_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR Calibration Mode\n" +
            "="*60 + "\n" +
            "  Adjust sensor positions via launch arguments:\n" +
            "    camera_x, camera_y, camera_z, camera_roll, camera_pitch, camera_yaw\n" +
            "    tof_x, tof_y, tof_z, tof_roll, tof_pitch, tof_yaw\n" +
            "    imu_x, imu_y, imu_z, imu_roll, imu_pitch, imu_yaw\n" +
            "="*60
    )
    
    return LaunchDescription([
        # Arguments
        camera_x_arg, camera_y_arg, camera_z_arg,
        camera_roll_arg, camera_pitch_arg, camera_yaw_arg,
        tof_x_arg, tof_y_arg, tof_z_arg,
        tof_roll_arg, tof_pitch_arg, tof_yaw_arg,
        imu_x_arg, imu_y_arg, imu_z_arg,
        imu_roll_arg, imu_pitch_arg, imu_yaw_arg,
        
        # Info
        info_msg,
        
        # Nodes
        robot_state_publisher,
        joint_state_publisher_gui,
        static_odom_tf,
        static_map_tf,
        rviz_node,
        calibration_helper,
    ])