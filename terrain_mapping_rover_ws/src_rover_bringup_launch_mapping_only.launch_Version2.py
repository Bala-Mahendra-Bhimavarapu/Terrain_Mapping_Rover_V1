"""
MAPPING ONLY - Build a map with manual driving

Launches SLAM without Nav2:
- Sensors + TF + EKF
- RTAB-Map SLAM
- Teleop for manual driving

Use this to build a map, then save it for later navigation. 

Usage:
    ros2 launch rover_bringup mapping_only.launch.py
    
Then drive around to build map. 

Save map: 
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, TimerAction
from launch. substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_bringup = get_package_share_directory('rover_bringup')
    
    use_mock = LaunchConfiguration('use_mock')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='false'),
        
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  MAPPING ONLY MODE'),
        LogInfo(msg='='*70),
        LogInfo(msg='\n  Build a map with manual driving.'),
        LogInfo(msg='  Includes:  Sensors + EKF + SLAM'),
        LogInfo(msg='  NO Nav2 - manual control only'),
        LogInfo(msg='\n  After mapping, save with:'),
        LogInfo(msg='    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map'),
        LogInfo(msg='\n' + '='*70 + '\n'),
        
        # Phase 4: SLAM (includes Phase 1-3)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'phase4_slam.launch.py')
            ),
            launch_arguments={'use_mock':  use_mock}.items()
        ),
        
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='\n' + '='*70),
                LogInfo(msg='  SLAM READY!  Drive around to build map.'),
                LogInfo(msg='  Run teleop:  ros2 run rover_teleop keyboard_teleop_node.py'),
                LogInfo(msg='='*70 + '\n'),
            ]
        ),
    ])