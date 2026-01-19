"""
PHASE 5: Perception Systems

This phase adds perception components:
- Auto-Exposure Controller (adjusts camera for optimal SLAM)
- YOLO Landmark Detector (detects objects)
- Landmark 3D Localizer (converts to 3D positions)
- Landmark Verification (checks for drift)

Prerequisites:
- Phase 4 (SLAM) must be running (includes Phase 1-3)

Definition of Done:
✅ /auto_exposure/state publishing
✅ /landmarks/detections publishing when objects visible
✅ /landmarks/positions publishing 3D positions
✅ /landmarks/markers visible in RViz
✅ Camera exposure adjusts automatically

Test Commands:
    ros2 topic echo /auto_exposure/state
    ros2 topic echo /landmarks/detections
    ros2 topic echo /landmarks/positions
    ros2 topic echo /landmarks/markers
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_bringup = get_package_share_directory('rover_bringup')
    pkg_auto_exp = get_package_share_directory('auto_exposure')
    pkg_landmark = get_package_share_directory('landmark_system')
    
    auto_exp_config = os.path.join(pkg_auto_exp, 'config', 'auto_exposure_params.yaml')
    landmark_config = os.path.join(pkg_landmark, 'config', 'landmark_params.yaml')
    
    use_mock = LaunchConfiguration('use_mock')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_auto_exposure = LaunchConfiguration('enable_auto_exposure')
    enable_landmarks = LaunchConfiguration('enable_landmarks')
    landmark_map_file = LaunchConfiguration('landmark_map_file')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_auto_exposure', default_value='true'),
        DeclareLaunchArgument('enable_landmarks', default_value='true'),
        DeclareLaunchArgument('landmark_map_file', default_value='',
            description='Path to landmark map file for verification'),
        
        # Startup message
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  PHASE 5: PERCEPTION SYSTEMS'),
        LogInfo(msg='='*70),
        LogInfo(msg='\n  Includes: Phase 1-4 + Perception'),
        LogInfo(msg='\n  Components:'),
        LogInfo(msg='    - Auto-Exposure Controller'),
        LogInfo(msg='    - YOLO Landmark Detector'),
        LogInfo(msg='    - Landmark 3D Localizer'),
        LogInfo(msg='    - Landmark Verification'),
        LogInfo(msg='\n  Test with:'),
        LogInfo(msg='    ros2 topic echo /auto_exposure/state'),
        LogInfo(msg='    ros2 topic echo /landmarks/detections'),
        LogInfo(msg='    ros2 topic echo /landmarks/positions'),
        LogInfo(msg='\n' + '='*70 + '\n'),
        
        # Phase 4: SLAM (includes Phase 1-3)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'phase4_slam.launch. py')
            ),
            launch_arguments={
                'use_mock': use_mock,
                'use_sim_time': use_sim_time
            }.items()
        ),
        
        # Perception systems (delay 6 seconds)
        TimerAction(
            period=6.0,
            actions=[
                LogInfo(msg='\n>>> Starting Perception Systems...\n'),
                
                # Auto-Exposure Controller
                Node(
                    package='auto_exposure',
                    executable='auto_exposure_node. py',
                    name='auto_exposure_node',
                    output='screen',
                    parameters=[auto_exp_config, {'use_sim_time': use_sim_time}],
                    condition=IfCondition(enable_auto_exposure)
                ),
                
                # YOLO Landmark Detector
                Node(
                    package='landmark_system',
                    executable='yolo_detector_node.py',
                    name='yolo_detector_node',
                    output='screen',
                    parameters=[landmark_config, {'use_sim_time':  use_sim_time}],
                    condition=IfCondition(enable_landmarks)
                ),
                
                # Landmark 3D Localizer
                Node(
                    package='landmark_system',
                    executable='landmark_localizer_node.py',
                    name='landmark_localizer_node',
                    output='screen',
                    parameters=[landmark_config, {'use_sim_time': use_sim_time}],
                    condition=IfCondition(enable_landmarks)
                ),
                
                # Landmark Verification
                Node(
                    package='landmark_system',
                    executable='landmark_verification_node.py',
                    name='landmark_verification_node',
                    output='screen',
                    parameters=[
                        landmark_config,
                        {'use_sim_time':  use_sim_time},
                        {'landmark_map_file': landmark_map_file}
                    ],
                    condition=IfCondition(enable_landmarks)
                ),
            ]
        ),
    ])