"""
PHASE 6: Nav2 Navigation Stack

This phase adds autonomous navigation:
- Path planning (SMAC Hybrid planner)
- Path following (Regulated Pure Pursuit controller)
- Obstacle avoidance (costmaps using ToF)
- Recovery behaviors (spin, backup)
- Waypoint following

Prerequisites:
- Phase 4 or 5 (SLAM) must be running

Definition of Done:
✅ Nav2 lifecycle manager reports all nodes active
✅ Can send goal via RViz "2D Goal Pose"
✅ Robot navigates to goal avoiding obstacles
✅ /plan topic shows planned path
✅ /local_costmap/costmap shows obstacles

Test Commands:
    ros2 topic echo /plan
    ros2 service call /lifecycle_manager_navigation/get_state
    
Send goal programmatically:
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}}}}"
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
    
    nav2_config = os.path. join(pkg_bringup, 'config', 'nav2_params.yaml')
    
    use_mock = LaunchConfiguration('use_mock')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_perception = LaunchConfiguration('enable_perception')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_perception', default_value='true',
            description='Include Phase 5 perception (auto-exposure, landmarks)'),
        
        # Startup message
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  PHASE 6: NAV2 NAVIGATION STACK'),
        LogInfo(msg='='*70),
        LogInfo(msg='\n  Includes: Phase 1-5 + Nav2'),
        LogInfo(msg='\n  Components:'),
        LogInfo(msg='    - Controller Server (path following)'),
        LogInfo(msg='    - Planner Server (path planning)'),
        LogInfo(msg='    - Behavior Server (recovery)'),
        LogInfo(msg='    - BT Navigator (behavior tree)'),
        LogInfo(msg='    - Costmaps (obstacle detection)'),
        LogInfo(msg='\n  Test with:'),
        LogInfo(msg='    - RViz:  Click "2D Goal Pose" and set goal'),
        LogInfo(msg='    - ros2 topic echo /plan'),
        LogInfo(msg='\n' + '='*70 + '\n'),
        
        # Phase 5: Perception (includes Phase 1-4) OR Phase 4 directly
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'phase5_perception.launch.py')
            ),
            launch_arguments={
                'use_mock': use_mock,
                'use_sim_time':  use_sim_time,
                'enable_auto_exposure': enable_perception,
                'enable_landmarks': enable_perception,
            }.items(),
            condition=IfCondition(enable_perception)
        ),
        
        # Phase 4 only (skip perception)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'phase4_slam. launch.py')
            ),
            launch_arguments={
                'use_mock': use_mock,
                'use_sim_time': use_sim_time
            }.items(),
            condition=IfCondition(
                PythonExpression(["not ", enable_perception])
            )
        ),
        
        # Nav2 Stack (delay 8 seconds)
        TimerAction(
            period=8.0,
            actions=[
                GroupAction([
                    LogInfo(msg='\n>>> Starting Nav2 Navigation Stack...\n'),
                    
                    # Controller Server
                    Node(
                        package='nav2_controller',
                        executable='controller_server',
                        name='controller_server',
                        output='screen',
                        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
                    ),
                    
                    # Planner Server
                    Node(
                        package='nav2_planner',
                        executable='planner_server',
                        name='planner_server',
                        output='screen',
                        parameters=[nav2_config, {'use_sim_time':  use_sim_time}],
                    ),
                    
                    # Behavior Server
                    Node(
                        package='nav2_behaviors',
                        executable='behavior_server',
                        name='behavior_server',
                        output='screen',
                        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
                    ),
                    
                    # BT Navigator
                    Node(
                        package='nav2_bt_navigator',
                        executable='bt_navigator',
                        name='bt_navigator',
                        output='screen',
                        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
                    ),
                    
                    # Waypoint Follower (Nav2's built-in)
                    Node(
                        package='nav2_waypoint_follower',
                        executable='waypoint_follower',
                        name='waypoint_follower',
                        output='screen',
                        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
                    ),
                    
                    # Velocity Smoother
                    Node(
                        package='nav2_velocity_smoother',
                        executable='velocity_smoother',
                        name='velocity_smoother',
                        output='screen',
                        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
                        remappings=[
                            ('cmd_vel', '/cmd_vel_nav'),
                            ('cmd_vel_smoothed', '/vex/cmd_vel')
                        ],
                    ),
                    
                    # Lifecycle Manager
                    Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_navigation',
                        output='screen',
                        parameters=[
                            {'use_sim_time': use_sim_time},
                            {'autostart': True},
                            {'node_names': [
                                'controller_server',
                                'planner_server',
                                'behavior_server',
                                'bt_navigator',
                                'waypoint_follower',
                                'velocity_smoother',
                            ]}
                        ],
                    ),
                ])
            ]
        ),
    ])