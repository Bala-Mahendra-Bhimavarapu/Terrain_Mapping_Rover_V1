"""
PHASE 8: Utilities

This phase adds utility components: 
- MJPEG Video Streaming (http://pi: 8080/stream)
- Web Dashboard (http://pi:5000)
- Bluetooth Beacon Scanner (optional)
- Beacon Homing System (optional)

Can run standalone or with any previous phase.

Usage:
    # Basic utilities
    ros2 launch rover_bringup phase8_utilities. launch.py
    
    # With beacon homing
    ros2 launch rover_bringup phase8_utilities.launch.py enable_beacon_homing:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_streaming = get_package_share_directory('rover_streaming')
    pkg_bluetooth = get_package_share_directory('bluetooth_nav')
    pkg_beacon_homing = get_package_share_directory('beacon_homing')
    
    streaming_config = os.path.join(pkg_streaming, 'config', 'streaming_params.yaml')
    bluetooth_config = os.path.join(pkg_bluetooth, 'config', 'bluetooth_params.yaml')
    beacon_homing_config = os. path.join(pkg_beacon_homing, 'config', 'beacon_homing_params. yaml')
    
    enable_streaming = LaunchConfiguration('enable_streaming')
    enable_dashboard = LaunchConfiguration('enable_dashboard')
    enable_bluetooth = LaunchConfiguration('enable_bluetooth')
    enable_beacon_homing = LaunchConfiguration('enable_beacon_homing')
    beacon_uuid = LaunchConfiguration('beacon_uuid')
    
    return LaunchDescription([
        DeclareLaunchArgument('enable_streaming', default_value='true'),
        DeclareLaunchArgument('enable_dashboard', default_value='true'),
        DeclareLaunchArgument('enable_bluetooth', default_value='false',
            description='Enable Bluetooth beacon scanner only'),
        DeclareLaunchArgument('enable_beacon_homing', default_value='false',
            description='Enable full beacon homing system (includes scanner)'),
        DeclareLaunchArgument('beacon_uuid', default_value='',
            description='Target beacon UUID'),
        
        # Startup message
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  PHASE 8: UTILITIES'),
        LogInfo(msg='='*70),
        LogInfo(msg='\n  Components:'),
        LogInfo(msg='    - MJPEG Streaming (port 8080)'),
        LogInfo(msg='    - Web Dashboard (port 5000)'),
        LogInfo(msg='    - Bluetooth Scanner (optional)'),
        LogInfo(msg='    - Beacon Homing (optional)'),
        LogInfo(msg='\n  Access:'),
        LogInfo(msg='    Camera: http://localhost:8080/stream'),
        LogInfo(msg='    Dashboard: http://localhost:5000'),
        LogInfo(msg='\n' + '='*70 + '\n'),
        
        GroupAction([
            # === MJPEG Streaming ===
            Node(
                package='rover_streaming',
                executable='mjpeg_streaming_node.py',
                name='mjpeg_streaming_node',
                output='screen',
                parameters=[streaming_config],
                condition=IfCondition(enable_streaming)
            ),
            
            # === Web Dashboard ===
            Node(
                package='web_dashboard',
                executable='dashboard_node.py',
                name='web_dashboard_node',
                output='screen',
                condition=IfCondition(enable_dashboard)
            ),
            
            # === Bluetooth Scanner (standalone or as part of beacon homing) ===
            Node(
                package='bluetooth_nav',
                executable='beacon_scanner_node.py',
                name='beacon_scanner_node',
                output='screen',
                parameters=[
                    bluetooth_config,
                    {'target_uuid': beacon_uuid}
                ],
                condition=IfCondition(enable_bluetooth)
            ),
            
            # === Beacon Homing System ===
            Node(
                package='bluetooth_nav',
                executable='beacon_scanner_node.py',
                name='beacon_scanner_node',
                output='screen',
                parameters=[
                    bluetooth_config,
                    {'target_uuid': beacon_uuid}
                ],
                condition=IfCondition(enable_beacon_homing)
            ),
            Node(
                package='beacon_homing',
                executable='beacon_homing_node.py',
                name='beacon_homing_node',
                output='screen',
                parameters=[
                    beacon_homing_config,
                    {'target_uuid': beacon_uuid}
                ],
                condition=IfCondition(enable_beacon_homing)
            ),
        ]),
    ])