"""
FULL SYSTEM LAUNCH - Everything for Complete Autonomous Operation

This launch file starts ALL components needed for full autonomous operation: 

PHASE 1: Hardware Drivers
  - VEX Serial (motor control + wheel odometry)
  - IMU (MPU6050)
  - RGB Camera (IMX500)
  - ToF Depth Camera (Arducam)

PHASE 2: Robot Description
  - URDF / Robot State Publisher
  - Static TF transforms

PHASE 3: Sensor Fusion
  - EKF (fuses wheel odometry + IMU)

PHASE 4: SLAM
  - RTAB-Map (RGB-D SLAM)

PHASE 5: Perception
  - Auto-Exposure Controller
  - YOLO Landmark Detection
  - Landmark 3D Localization
  - Landmark Verification

PHASE 6: Navigation
  - Nav2 (path planning + obstacle avoidance)

PHASE 7: Mission Management
  - Mission Health Monitor
  - Waypoint Recorder
  - Waypoint Follower

PHASE 8: Utilities
  - MJPEG Streaming
  - Web Dashboard
  - Bluetooth Beacon Scanner (optional)

Author:  Rover Team
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
    LogInfo,
    RegisterEventHandler
)
from launch.conditions import IfCondition, UnlessCondition
from launch. substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch. event_handlers import OnProcessStart
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # =========================================================================
    # PACKAGE DIRECTORIES
    # =========================================================================
    pkg_bringup = get_package_share_directory('rover_bringup')
    pkg_description = get_package_share_directory('rover_description')
    pkg_vex = get_package_share_directory('vex_serial')
    pkg_imu = get_package_share_directory('imu_driver')
    pkg_camera = get_package_share_directory('imx500_camera')
    pkg_tof = get_package_share_directory('arducam_tof')
    pkg_fusion = get_package_share_directory('sensor_fusion')
    pkg_auto_exp = get_package_share_directory('auto_exposure')
    pkg_health = get_package_share_directory('mission_health')
    pkg_waypoint = get_package_share_directory('waypoint_manager')
    pkg_landmark = get_package_share_directory('landmark_system')
    pkg_streaming = get_package_share_directory('rover_streaming')
    pkg_bluetooth = get_package_share_directory('bluetooth_nav')
    pkg_dashboard = get_package_share_directory('web_dashboard')
    
    # =========================================================================
    # CONFIGURATION FILES
    # =========================================================================
    vex_config = os.path.join(pkg_vex, 'config', 'vex_serial_params.yaml')
    imu_config = os.path.join(pkg_imu, 'config', 'mpu6050_params.yaml')
    camera_config = os. path.join(pkg_camera, 'config', 'imx500_params.yaml')
    tof_config = os.path.join(pkg_tof, 'config', 'arducam_tof_params.yaml')
    ekf_config = os.path. join(pkg_fusion, 'config', 'ekf_params.yaml')
    auto_exp_config = os.path.join(pkg_auto_exp, 'config', 'auto_exposure_params.yaml')
    health_config = os.path.join(pkg_health, 'config', 'mission_health_params.yaml')
    waypoint_config = os.path.join(pkg_waypoint, 'config', 'waypoint_params. yaml')
    landmark_config = os.path.join(pkg_landmark, 'config', 'landmark_params.yaml')
    streaming_config = os.path. join(pkg_streaming, 'config', 'streaming_params.yaml')
    bluetooth_config = os.path.join(pkg_bluetooth, 'config', 'bluetooth_params.yaml')
    rtabmap_config = os.path. join(pkg_bringup, 'config', 'rtabmap_params.yaml')
    nav2_config = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    
    # =========================================================================
    # LAUNCH ARGUMENTS
    # =========================================================================
    declared_arguments = [
        # === SIMULATION / HARDWARE MODE ===
        DeclareLaunchArgument(
            'use_mock', 
            default_value='false',
            description='Use mock sensor nodes (for testing without hardware)'
        ),
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false',
            description='Use simulation time'
        ),
        
        # === FEATURE TOGGLES ===
        DeclareLaunchArgument(
            'enable_slam', 
            default_value='true',
            description='Enable RTAB-Map SLAM'
        ),
        DeclareLaunchArgument(
            'enable_nav2', 
            default_value='true',
            description='Enable Nav2 navigation stack'
        ),
        DeclareLaunchArgument(
            'enable_auto_exposure', 
            default_value='true',
            description='Enable auto-exposure controller'
        ),
        DeclareLaunchArgument(
            'enable_landmarks', 
            default_value='true',
            description='Enable YOLO landmark detection and localization'
        ),
        DeclareLaunchArgument(
            'enable_waypoints', 
            default_value='true',
            description='Enable waypoint recording and following'
        ),
        DeclareLaunchArgument(
            'enable_health_monitor', 
            default_value='true',
            description='Enable mission health monitoring'
        ),
        DeclareLaunchArgument(
            'enable_streaming', 
            default_value='true',
            description='Enable MJPEG video streaming'
        ),
        DeclareLaunchArgument(
            'enable_dashboard', 
            default_value='true',
            description='Enable web dashboard'
        ),
        DeclareLaunchArgument(
            'enable_bluetooth', 
            default_value='false',
            description='Enable Bluetooth beacon navigation'
        ),
        
        # === OPTIONAL MAP FILE (for localization-only mode) ===
        DeclareLaunchArgument(
            'map_file', 
            default_value='',
            description='Path to pre-built map file (empty = build new map)'
        ),
        
        # === LANDMARK MAP FILE ===
        DeclareLaunchArgument(
            'landmark_map_file', 
            default_value='',
            description='Path to landmark map file for verification'
        ),
    ]
    
    # Get launch configurations
    use_mock = LaunchConfiguration('use_mock')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_slam = LaunchConfiguration('enable_slam')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    enable_auto_exposure = LaunchConfiguration('enable_auto_exposure')
    enable_landmarks = LaunchConfiguration('enable_landmarks')
    enable_waypoints = LaunchConfiguration('enable_waypoints')
    enable_health_monitor = LaunchConfiguration('enable_health_monitor')
    enable_streaming = LaunchConfiguration('enable_streaming')
    enable_dashboard = LaunchConfiguration('enable_dashboard')
    enable_bluetooth = LaunchConfiguration('enable_bluetooth')
    map_file = LaunchConfiguration('map_file')
    landmark_map_file = LaunchConfiguration('landmark_map_file')
    
    # =========================================================================
    # PHASE 1: HARDWARE DRIVERS (Start immediately)
    # =========================================================================
    phase1_hardware = GroupAction([
        LogInfo(msg='\n========== PHASE 1: Starting Hardware Drivers ==========\n'),
        
        # --- VEX Serial Node ---
        Node(
            package='vex_serial',
            executable='vex_serial_node. py',
            name='vex_serial_node',
            output='screen',
            parameters=[vex_config, {'use_sim_time': use_sim_time}],
            condition=UnlessCondition(use_mock)
        ),
        Node(
            package='vex_serial',
            executable='vex_mock_node.py',
            name='vex_serial_node',
            output='screen',
            parameters=[vex_config, {'use_sim_time': use_sim_time}],
            condition=IfCondition(use_mock)
        ),
        
        # --- IMU Node ---
        Node(
            package='imu_driver',
            executable='mpu6050_node.py',
            name='mpu6050_node',
            output='screen',
            parameters=[imu_config, {'use_sim_time': use_sim_time}],
            condition=UnlessCondition(use_mock)
        ),
        Node(
            package='imu_driver',
            executable='imu_mock_node.py',
            name='mpu6050_node',
            output='screen',
            parameters=[imu_config, {'use_sim_time': use_sim_time}],
            condition=IfCondition(use_mock)
        ),
        
        # --- RGB Camera Node ---
        Node(
            package='imx500_camera',
            executable='imx500_camera_node.py',
            name='imx500_camera_node',
            output='screen',
            parameters=[camera_config, {'use_sim_time': use_sim_time}],
            condition=UnlessCondition(use_mock)
        ),
        Node(
            package='imx500_camera',
            executable='camera_mock_node.py',
            name='imx500_camera_node',
            output='screen',
            parameters=[camera_config, {'use_sim_time':  use_sim_time}],
            condition=IfCondition(use_mock)
        ),
        
        # --- ToF Depth Camera Node ---
        Node(
            package='arducam_tof',
            executable='arducam_tof_node. py',
            name='arducam_tof_node',
            output='screen',
            parameters=[tof_config, {'use_sim_time': use_sim_time}],
            condition=UnlessCondition(use_mock)
        ),
        Node(
            package='arducam_tof',
            executable='tof_mock_node.py',
            name='arducam_tof_node',
            output='screen',
            parameters=[tof_config, {'use_sim_time': use_sim_time}],
            condition=IfCondition(use_mock)
        ),
    ])
    
    # =========================================================================
    # PHASE 2: ROBOT DESCRIPTION & TF (Start immediately)
    # =========================================================================
    phase2_description = GroupAction([
        LogInfo(msg='\n========== PHASE 2: Starting Robot Description ==========\n'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_description, 'launch', 'description.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
    ])
    
    # =========================================================================
    # PHASE 3: SENSOR FUSION - EKF (Delay 2 seconds for sensors to start)
    # =========================================================================
    phase3_fusion = TimerAction(
        period=2.0,
        actions=[
            GroupAction([
                LogInfo(msg='\n========== PHASE 3: Starting EKF Sensor Fusion ==========\n'),
                
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[ekf_config, {'use_sim_time': use_sim_time}],
                    remappings=[
                        ('/odometry/filtered', '/odom'),
                    ]
                ),
            ])
        ]
    )
    
    # =========================================================================
    # PHASE 4: SLAM - RTAB-Map (Delay 4 seconds for EKF to stabilize)
    # =========================================================================
    phase4_slam = TimerAction(
        period=4.0,
        actions=[
            GroupAction([
                LogInfo(msg='\n========== PHASE 4: Starting RTAB-Map SLAM ==========\n'),
                
                # RTAB-Map node
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
                    arguments=[
                        '--delete_db_on_start',  # Remove for persistent mapping
                    ],
                    condition=IfCondition(enable_slam)
                ),
                
                # RTAB-Map visualization (publishes map -> odom transform)
                Node(
                    package='rtabmap_viz',
                    executable='rtabmap_viz',
                    name='rtabmap_viz',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    condition=IfCondition(enable_slam)
                ),
            ])
        ]
    )
    
    # =========================================================================
    # PHASE 5: PERCEPTION (Delay 5 seconds)
    # =========================================================================
    phase5_perception = TimerAction(
        period=5.0,
        actions=[
            GroupAction([
                LogInfo(msg='\n========== PHASE 5: Starting Perception Systems ==========\n'),
                
                # --- Auto-Exposure Controller ---
                Node(
                    package='auto_exposure',
                    executable='auto_exposure_node.py',
                    name='auto_exposure_node',
                    output='screen',
                    parameters=[auto_exp_config, {'use_sim_time': use_sim_time}],
                    condition=IfCondition(enable_auto_exposure)
                ),
                
                # --- YOLO Landmark Detector ---
                Node(
                    package='landmark_system',
                    executable='yolo_detector_node.py',
                    name='yolo_detector_node',
                    output='screen',
                    parameters=[landmark_config, {'use_sim_time':  use_sim_time}],
                    condition=IfCondition(enable_landmarks)
                ),
                
                # --- Landmark 3D Localizer ---
                Node(
                    package='landmark_system',
                    executable='landmark_localizer_node.py',
                    name='landmark_localizer_node',
                    output='screen',
                    parameters=[landmark_config, {'use_sim_time': use_sim_time}],
                    condition=IfCondition(enable_landmarks)
                ),
                
                # --- Landmark Verification ---
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
            ])
        ]
    )
    
    # =========================================================================
    # PHASE 6: NAVIGATION - Nav2 (Delay 6 seconds for SLAM to initialize)
    # =========================================================================
    phase6_navigation = TimerAction(
        period=6.0,
        actions=[
            GroupAction([
                LogInfo(msg='\n========== PHASE 6: Starting Nav2 Navigation ==========\n'),
                
                # --- Nav2 Lifecycle Manager for Localization ---
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[nav2_config, {'use_sim_time': use_sim_time}],
                    condition=IfCondition(enable_nav2)
                ),
                
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[nav2_config, {'use_sim_time': use_sim_time}],
                    condition=IfCondition(enable_nav2)
                ),
                
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[nav2_config, {'use_sim_time': use_sim_time}],
                    condition=IfCondition(enable_nav2)
                ),
                
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[nav2_config, {'use_sim_time': use_sim_time}],
                    condition=IfCondition(enable_nav2)
                ),
                
                Node(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    output='screen',
                    parameters=[nav2_config, {'use_sim_time':  use_sim_time}],
                    condition=IfCondition(enable_nav2)
                ),
                
                Node(
                    package='nav2_velocity_smoother',
                    executable='velocity_smoother',
                    name='velocity_smoother',
                    output='screen',
                    parameters=[nav2_config, {'use_sim_time':  use_sim_time}],
                    remappings=[
                        ('cmd_vel', '/cmd_vel_nav'),
                        ('cmd_vel_smoothed', '/vex/cmd_vel')
                    ],
                    condition=IfCondition(enable_nav2)
                ),
                
                # --- Costmap Nodes ---
                Node(
                    package='nav2_costmap_2d',
                    executable='nav2_costmap_2d',
                    name='local_costmap',
                    output='screen',
                    parameters=[nav2_config, {'use_sim_time': use_sim_time}],
                    condition=IfCondition(enable_nav2)
                ),
                
                Node(
                    package='nav2_costmap_2d',
                    executable='nav2_costmap_2d',
                    name='global_costmap',
                    output='screen',
                    parameters=[nav2_config, {'use_sim_time': use_sim_time}],
                    condition=IfCondition(enable_nav2)
                ),
                
                # --- Nav2 Lifecycle Manager ---
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
                    condition=IfCondition(enable_nav2)
                ),
            ])
        ]
    )
    
    # =========================================================================
    # PHASE 7: MISSION MANAGEMENT (Delay 8 seconds)
    # =========================================================================
    phase7_mission = TimerAction(
        period=8.0,
        actions=[
            GroupAction([
                LogInfo(msg='\n========== PHASE 7: Starting Mission Management ==========\n'),
                
                # --- Mission Health Monitor ---
                Node(
                    package='mission_health',
                    executable='mission_health_node.py',
                    name='mission_health_node',
                    output='screen',
                    parameters=[health_config, {'use_sim_time': use_sim_time}],
                    condition=IfCondition(enable_health_monitor)
                ),
                
                # --- Waypoint Recorder ---
                Node(
                    package='waypoint_manager',
                    executable='waypoint_recorder_node.py',
                    name='waypoint_recorder_node',
                    output='screen',
                    parameters=[waypoint_config, {'use_sim_time': use_sim_time}],
                    condition=IfCondition(enable_waypoints)
                ),
                
                # --- Waypoint Follower ---
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
    )
    
    # =========================================================================
    # PHASE 8: UTILITIES (Delay 10 seconds)
    # =========================================================================
    phase8_utilities = TimerAction(
        period=10.0,
        actions=[
            GroupAction([
                LogInfo(msg='\n========== PHASE 8: Starting Utilities ==========\n'),
                
                # --- MJPEG Streaming ---
                Node(
                    package='rover_streaming',
                    executable='mjpeg_streaming_node.py',
                    name='mjpeg_streaming_node',
                    output='screen',
                    parameters=[streaming_config],
                    condition=IfCondition(enable_streaming)
                ),
                
                # --- Web Dashboard ---
                Node(
                    package='web_dashboard',
                    executable='dashboard_node.py',
                    name='web_dashboard_node',
                    output='screen',
                    condition=IfCondition(enable_dashboard)
                ),
                
                # --- Bluetooth Beacon Scanner ---
                Node(
                    package='bluetooth_nav',
                    executable='beacon_scanner_node.py',
                    name='beacon_scanner_node',
                    output='screen',
                    parameters=[bluetooth_config],
                    condition=IfCondition(enable_bluetooth)
                ),
                
                # --- Bluetooth Navigator ---
                Node(
                    package='bluetooth_nav',
                    executable='beacon_navigator_node.py',
                    name='beacon_navigator_node',
                    output='screen',
                    parameters=[bluetooth_config, {'enabled': False}],  # Must enable via param
                    condition=IfCondition(enable_bluetooth)
                ),
            ])
        ]
    )
    
    # =========================================================================
    # STARTUP COMPLETE MESSAGE
    # =========================================================================
    startup_complete = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='\n' + '='*70),
            LogInfo(msg='  🚀 LUNAR ROVER FULL SYSTEM STARTUP COMPLETE!  🌙'),
            LogInfo(msg='='*70),
            LogInfo(msg='\n  Web Dashboard:     http://localhost:5000'),
            LogInfo(msg='  Camera Stream:    http://localhost:8080/stream'),
            LogInfo(msg='\n  Key Services:'),
            LogInfo(msg='    - Start waypoint recording: '),
            LogInfo(msg='      ros2 service call /waypoint_recorder/start std_srvs/srv/SetBool "{data: true}"'),
            LogInfo(msg='    - Stop waypoint recording:'),
            LogInfo(msg='      ros2 service call /waypoint_recorder/stop std_srvs/srv/Trigger'),
            LogInfo(msg='    - Follow waypoints:'),
            LogInfo(msg='      ros2 service call /waypoint_follower/start std_srvs/srv/Trigger'),
            LogInfo(msg='\n' + '='*70 + '\n'),
        ]
    )
    
    # =========================================================================
    # RETURN LAUNCH DESCRIPTION
    # =========================================================================
    return LaunchDescription(
        declared_arguments + [
            # Phase 1 & 2: Start immediately
            phase1_hardware,
            phase2_description,
            
            # Phase 3: EKF (2 second delay)
            phase3_fusion,
            
            # Phase 4: SLAM (4 second delay)
            phase4_slam,
            
            # Phase 5: Perception (5 second delay)
            phase5_perception,
            
            # Phase 6: Navigation (6 second delay)
            phase6_navigation,
            
            # Phase 7: Mission Management (8 second delay)
            phase7_mission,
            
            # Phase 8: Utilities (10 second delay)
            phase8_utilities,
            
            # Startup complete message
            startup_complete,
        ]
    )