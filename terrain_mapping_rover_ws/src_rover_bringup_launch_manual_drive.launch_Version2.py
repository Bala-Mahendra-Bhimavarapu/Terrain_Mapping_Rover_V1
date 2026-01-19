"""
MANUAL DRIVE - Minimal system for teleop driving

Launches only what's needed for manual control:
- Sensors (VEX, IMU)
- TF
- EKF
- Teleop (in separate terminal)

NO SLAM, NO NAV2 - just drive around and test odometry. 

Usage:
    ros2 launch rover_bringup manual_drive. launch. py
    
Then in another terminal:
    ros2 run rover_teleop keyboard_teleop_node. py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_bringup = get_package_share_directory('rover_bringup')
    pkg_teleop = get_package_share_directory('rover_teleop')
    
    teleop_config = os.path. join(pkg_teleop, 'config', 'teleop_params.yaml')
    
    use_mock = LaunchConfiguration('use_mock')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='false'),
        
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  MANUAL DRIVE MODE'),
        LogInfo(msg='='*70),
        LogInfo(msg='\n  Minimal system for teleop driving. '),
        LogInfo(msg='  NO SLAM, NO NAV2 - just sensors + EKF + teleop'),
        LogInfo(msg='\n  Controls:'),
        LogInfo(msg='    W/S - Forward/Back'),
        LogInfo(msg='    A/D - Turn Left/Right'),
        LogInfo(msg='    Space - Stop'),
        LogInfo(msg='    +/- - Adjust speed'),
        LogInfo(msg='\n' + '='*70 + '\n'),
        
        # Phase 3: EKF (includes Phase 1 & 2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'phase3_ekf.launch.py')
            ),
            launch_arguments={'use_mock': use_mock}. items()
        ),
        
        # Teleop hint (don't actually launch in terminal - user does it)
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='\n' + '='*70),
                LogInfo(msg='  READY FOR TELEOP! '),
                LogInfo(msg='  In another terminal, run:'),
                LogInfo(msg='    ros2 run rover_teleop keyboard_teleop_node.py'),
                LogInfo(msg='='*70 + '\n'),
            ]
        ),
    ])