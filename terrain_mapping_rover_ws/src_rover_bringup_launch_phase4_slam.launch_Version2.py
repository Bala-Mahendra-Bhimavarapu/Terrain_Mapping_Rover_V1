"""
PHASE 4: RTAB-Map SLAM

This phase adds visual SLAM:
- RTAB-Map RGB-D SLAM
- Builds occupancy grid map
- Publishes map → odom transform

Prerequisites:
- Phase 3 (EKF) must be running (includes Phase 1 & 2)

Definition of Done:
✅ /map topic publishing (OccupancyGrid)
✅ TF: map → odom available
✅ Map builds as robot moves
✅ Loop closures detected when revisiting areas
✅ Map visible in RViz

Test Commands:
    ros2 topic hz /map
    ros2 topic echo /rtabmap/info
    ros2 run tf2_ros tf2_echo map odom
    
Save map:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
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
    
    rtabmap_config = os.path.join(pkg_bringup, 'config', 'rtabmap_params.yaml')
    
    use_mock = LaunchConfiguration('use_mock')
    use_sim_time = LaunchConfiguration('use_sim_time')
    delete_db = LaunchConfiguration('delete_db')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'delete_db',
            default_value='true',
            description='Delete RTAB-Map database on start (false to resume mapping)'
        ),
        
        # Startup message
        LogInfo(msg='\n' + '='*70),
        LogInfo(msg='  PHASE 4: RTAB-MAP SLAM'),
        LogInfo(msg='='*70),
        LogInfo(msg='\n  Includes: Phase 1-3 + RTAB-Map SLAM'),
        LogInfo(msg='\n  Test with:'),
        LogInfo(msg='    ros2 topic hz /map'),
        LogInfo(msg='    ros2 run tf2_ros tf2_echo map odom'),
        LogInfo(msg='\n  View in RViz:'),
        LogInfo(msg='    Add Map display, topic: /map'),
        LogInfo(msg='    Add PointCloud2 display, topic: /rtabmap/cloud_map'),
        LogInfo(msg='\n  Save map:'),
        LogInfo(msg='    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map'),
        LogInfo(msg='\n' + '='*70 + '\n'),
        
        # Phase 3: EKF (includes Phase 1 & 2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'phase3_ekf. launch.py')
            ),
            launch_arguments={
                'use_mock': use_mock,
                'use_sim_time': use_sim_time
            }.items()
        ),
        
        # RTAB-Map (delay 4 seconds for EKF to stabilize)
        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg='\n>>> Starting RTAB-Map SLAM.. .\n'),
                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    name='rtabmap',
                    output='screen',
                    parameters=[
                        rtabmap_config,
                        {'use_sim_time': use_sim_time}
                    ],
                    remappings=[
                        ('rgb/image', '/camera/image_raw'),
                        ('rgb/camera_info', '/camera/camera_info'),
                        ('depth/image', '/tof/depth/image_raw'),
                        ('odom', '/odom'),
                    ],
                    arguments=['--delete_db_on_start'] if delete_db else [],
                ),
            ]
        ),
    ])