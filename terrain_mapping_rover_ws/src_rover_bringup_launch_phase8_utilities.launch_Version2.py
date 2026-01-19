"""
PHASE 8: Utilities

This phase adds utility components:
- MJPEG Video Streaming (http://pi: 8080/stream)
- Web Dashboard (http://pi:5000)
- Bluetooth Beacon Scanner (optional)
- Bluetooth Navigator (optional)

Prerequisites:
- Can run standalone or with any previous phase

Definition of Done: 
✅ Camera stream accessible at http://<pi-ip>:8080/stream
✅ Web dashboard accessible at http://<pi-ip>:5000
✅ Dashboard shows real-time sensor data
✅ (Optional) Bluetooth beacons detected

Test Commands:
    curl http://localhost:5000           # Dashboard
    curl http://localhost:8080/snapshot  # Camera snapshot
    ros2 topic echo /bluetooth/beacons   # Beacons
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
    
    streaming_config = os.path.join(pkg_streaming, 'config', 'streaming_params.yaml')
    bluetooth_config = os.path.join(pkg_bluetooth, 'config', 'bluetooth_params.yaml')
    
    enable_streaming = LaunchConfiguration('enable_streaming')
    enable_dashboard = LaunchConfiguration('enable_dashboard')
    enable_bluetooth = LaunchConfiguration('enable_bluetooth')
    
    return LaunchDescription([
        DeclareLaunchArgument('enable_streaming', default_value='true'),
        DeclareLaunchArgument('enable_dashboard', default_value='true'),
        DeclareLaunchArgument('enable_bluetooth', default_value='false'),
        
        # Startup message
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  PHASE 8: UTILITIES'),
        LogInfo(msg='='*70),
        LogInfo(msg='\n  Components:'),
        LogInfo(msg='    - MJPEG Streaming (port 8080)'),
        LogInfo(msg='    - Web Dashboard (port 5000)'),
        LogInfo(msg='    - Bluetooth Scanner (optional)'),
        LogInfo(msg='\n  Access:'),
        LogInfo(msg='    Camera: http://localhost:8080/stream'),
        LogInfo(msg='    Dashboard: http://localhost:5000'),
        LogInfo(msg='\n' + '='*70 + '\n'),
        
        GroupAction([
            # MJPEG Streaming
            Node(
                package='rover_streaming',
                executable='mjpeg_streaming_node.py',
                name='mjpeg_streaming_node',
                output='screen',
                parameters=[streaming_config],
                condition=IfCondition(enable_streaming)
            ),
            
            # Web Dashboard
            Node(
                package='web_dashboard',
                executable='dashboard_node.py',
                name='web_dashboard_node',
                output='screen',
                condition=IfCondition(enable_dashboard)
            ),
            
            # Bluetooth Scanner
            Node(
                package='bluetooth_nav',
                executable='beacon_scanner_node.py',
                name='beacon_scanner_node',
                output='screen',
                parameters=[bluetooth_config],
                condition=IfCondition(enable_bluetooth)
            ),
            
            # Bluetooth Navigator
            Node(
                package='bluetooth_nav',
                executable='beacon_navigator_node.py',
                name='beacon_navigator_node',
                output='screen',
                parameters=[bluetooth_config, {'enabled': False}],
                condition=IfCondition(enable_bluetooth)
            ),
        ]),
    ])