"""
PHASE 2: TF Tree Setup

This phase sets up the robot description and TF tree:
- URDF model loading
- Robot State Publisher
- Static transforms for all sensor frames

Definition of Done:
✅ TF tree visible with:  ros2 run tf2_tools view_frames
✅ All frames connected properly: 
   - base_link (robot center)
   - base_footprint (on ground)
   - camera_link → camera_optical_frame
   - tof_link → tof_optical_frame
   - imu_link
   - wheel frames (fl, fr, bl, br)
✅ No TF errors in terminal
✅ RViz shows robot model correctly

Test Commands:
    ros2 run tf2_tools view_frames
    ros2 run tf2_ros tf2_echo base_link camera_link
    ros2 run tf2_ros tf2_echo base_link imu_link
    ros2 topic echo /robot_description --once
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch. launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_description = get_package_share_directory('rover_description')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Startup message
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  PHASE 2: TF TREE SETUP'),
        LogInfo(msg='='*70),
        LogInfo(msg='\n  Starting: Robot State Publisher (URDF + TF)'),
        LogInfo(msg='\n  Test with:'),
        LogInfo(msg='    ros2 run tf2_tools view_frames'),
        LogInfo(msg='    ros2 run tf2_ros tf2_echo base_link camera_link'),
        LogInfo(msg='    ros2 run tf2_ros tf2_echo base_link imu_link'),
        LogInfo(msg='    ros2 run tf2_ros tf2_echo base_link tof_link'),
        LogInfo(msg='\n  View in RViz: '),
        LogInfo(msg='    rviz2 -d <path>/rover_default.rviz'),
        LogInfo(msg='\n' + '='*70 + '\n'),
        
        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_description, 'launch', 'description.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
    ])