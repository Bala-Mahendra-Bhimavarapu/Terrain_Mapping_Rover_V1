"""
WAYPOINT MISSION - Record and replay waypoints

Full system optimized for waypoint-based missions:
- All sensors + SLAM + Nav2
- Waypoint recording enabled
- Mission health monitoring

Workflow:
1. Launch this file
2. Start recording:  ros2 service call /waypoint_recorder/start ... 
3. Drive the route manually
4. Stop recording: ros2 service call /waypoint_recorder/stop ... 
5. Replay: ros2 service call /waypoint_follower/start ... 

Usage:
    ros2 launch rover_bringup waypoint_mission.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_bringup = get_package_share_directory('rover_bringup')
    
    use_mock = LaunchConfiguration('use_mock')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='false'),
        
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  WAYPOINT MISSION MODE'),
        LogInfo(msg='='*70),
        LogInfo(msg='\n  Full system for waypoint recording and replay.'),
        LogInfo(msg='\n  Quick Commands:'),
        LogInfo(msg='    START RECORDING (AUTO mode):'),
        LogInfo(msg='      ros2 service call /waypoint_recorder/start std_srvs/srv/SetBool "{data: true}"'),
        LogInfo(msg='    STOP RECORDING:'),
        LogInfo(msg='      ros2 service call /waypoint_recorder/stop std_srvs/srv/Trigger'),
        LogInfo(msg='    FOLLOW WAYPOINTS:'),
        LogInfo(msg='      ros2 service call /waypoint_follower/start std_srvs/srv/Trigger'),
        LogInfo(msg='    CHECK STATUS:'),
        LogInfo(msg='      ros2 topic echo /mission_health'),
        LogInfo(msg='\n' + '='*70 + '\n'),
        
        # Phase 7: Mission Management (includes Phase 1-6)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'phase7_mission.launch.py')
            ),
            launch_arguments={'use_mock':  use_mock}.items()
        ),
    ])