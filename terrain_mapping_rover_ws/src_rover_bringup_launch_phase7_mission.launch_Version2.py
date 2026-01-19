"""
PHASE 7: Mission Management

This phase adds mission management components:
- Mission Health Monitor (system health tracking)
- Waypoint Recorder (record poses during driving)
- Waypoint Follower (replay recorded waypoints)

Prerequisites:
- Phase 6 (Nav2) should be running for waypoint following
- Can run Phase 4+ for recording-only

Definition of Done:
✅ /mission_health publishing system status
✅ Can start/stop waypoint recording via services
✅ Waypoints saved to ~/. ros/waypoints/
✅ Can load and follow saved waypoints

Test Commands:
    ros2 topic echo /mission_health
    ros2 service call /waypoint_recorder/start std_srvs/srv/SetBool "{data: true}"
    ros2 service call /waypoint_recorder/stop std_srvs/srv/Trigger
    ros2 service call /waypoint_follower/load std_srvs/srv/Trigger
    ros2 service call /waypoint_follower/start std_srvs/srv/Trigger
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, TimerAction, GroupAction
from launch.conditions import IfCondition
from launch. substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_bringup = get_package_share_directory('rover_bringup')
    pkg_health = get_package_share_directory('mission_health')
    pkg_waypoint = get_package_share_directory('waypoint_manager')
    
    health_config = os.path.join(pkg_health, 'config', 'mission_health_params.yaml')
    waypoint_config = os.path.join(pkg_waypoint, 'config', 'waypoint_params. yaml')
    
    use_mock = LaunchConfiguration('use_mock')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    enable_health = LaunchConfiguration('enable_health')
    enable_waypoints = LaunchConfiguration('enable_waypoints')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_nav2', default_value='true',
            description='Include Nav2 (required for waypoint following)'),
        DeclareLaunchArgument('enable_health', default_value='true'),
        DeclareLaunchArgument('enable_waypoints', default_value='true'),
        
        # Startup message
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  PHASE 7: MISSION MANAGEMENT'),
        LogInfo(msg='='*70),
        LogInfo(msg='\n  Includes: Phase 1-6 + Mission Management'),
        LogInfo(msg='\n  Components:'),
        LogInfo(msg='    - Mission Health Monitor'),
        LogInfo(msg='    - Waypoint Recorder'),
        LogInfo(msg='    - Waypoint Follower'),
        LogInfo(msg='\n  Waypoint Commands:'),
        LogInfo(msg='    Start recording: '),
        LogInfo(msg='      ros2 service call /waypoint_recorder/start std_srvs/srv/SetBool "{data: true}"'),
        LogInfo(msg='    Stop recording: '),
        LogInfo(msg='      ros2 service call /waypoint_recorder/stop std_srvs/srv/Trigger'),
        LogInfo(msg='    Follow waypoints:'),
        LogInfo(msg='      ros2 service call /waypoint_follower/start std_srvs/srv/Trigger'),
        LogInfo(msg='\n' + '='*70 + '\n'),
        
        # Phase 6: Nav2 (includes Phase 1-5)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'phase6_nav2.launch.py')
            ),
            launch_arguments={
                'use_mock': use_mock,
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(enable_nav2)
        ),
        
        # Phase 4:  SLAM only (skip Nav2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'phase4_slam.launch.py')
            ),
            launch_arguments={
                'use_mock': use_mock,
                'use_sim_time': use_sim_time
            }.items(),
            condition=IfCondition(
                PythonExpression(["not ", enable_nav2])
            )
        ),
        
        # Mission Management (delay 10 seconds)
        TimerAction(
            period=10.0,
            actions=[
                GroupAction([
                    LogInfo(msg='\n>>> Starting Mission Management...\n'),
                    
                    # Mission Health Monitor
                    Node(
                        package='mission_health',
                        executable='mission_health_node.py',
                        name='mission_health_node',
                        output='screen',
                        parameters=[health_config, {'use_sim_time': use_sim_time}],
                        condition=IfCondition(enable_health)
                    ),
                    
                    # Waypoint Recorder
                    Node(
                        package='waypoint_manager',
                        executable='waypoint_recorder_node.py',
                        name='waypoint_recorder_node',
                        output='screen',
                        parameters=[waypoint_config, {'use_sim_time': use_sim_time}],
                        condition=IfCondition(enable_waypoints)
                    ),
                    
                    # Waypoint Follower (our custom node)
                    Node(
                        package='waypoint_manager',
                        executable='waypoint_follower_node.py',
                        name='waypoint_follower_node',
                        output='screen',
                        parameters=[waypoint_config, {'use_sim_time': use_sim_time}],
                        condition=IfCondition(enable_waypoints)
                    ),
                ])
            ]
        ),
    ])