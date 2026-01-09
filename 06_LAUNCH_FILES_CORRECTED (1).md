# PHASE LAUNCH FILES - Complete Set
# Place in: src/rover_launch/launch/

This file contains all 11 phase launch files + full mission launch

---

## phase_0_workspace_setup.launch.py

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Just verify packages are built
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            name='map_to_odom_static'
        )
    ])
```

---

## phase_1_sensors.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch all 4 sensor drivers independently
    return LaunchDescription([
        # VEX driver
        Node(
            package='vex_driver_node',
            executable='vex_driver_node',
            name='vex_driver',
            parameters=[{'serial_port': '/dev/ttyACM0',
                        'baud_rate': 115200}],
            output='screen'
        ),
        
        # IMU driver
        Node(
            package='imu_driver_node',
            executable='imu_driver_node',
            name='imu_driver',
            parameters=[{'i2c_device': '/dev/i2c-1',
                        'publish_frequency': 100}],
            output='screen'
        ),
        
        # IMX500 RGB camera
        Node(
            package='imx500_camera_node',
            executable='imx500_camera_node',
            name='imx500_camera',
            parameters=[{'resolution_width': 640,
                        'resolution_height': 480,
                        'fps': 30}],
            output='screen'
        ),
        
        # ToF depth camera
        Node(
            package='tof_camera_node',
            executable='tof_camera_node',
            name='tof_camera',
            parameters=[{'resolution_width': 320,
                        'resolution_height': 240,
                        'fps': 15}],
            output='screen'
        ),
    ])
```

**IMPORTANT: Start serial bridge FIRST in Terminal 1:**
```bash
ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
# Wait for: [INFO] ROS Serial Client initialized
```

**Test command (Terminal 2):**
```bash
ros2 launch rover_launch phase_1_sensors.launch.py
# In another terminal:
ros2 topic list  # Should see /vex/odom_raw, /imu/data, /image_raw, /tof/depth/image_raw
ros2 topic echo /vex/odom_raw
```

---

## phase_2_tf.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static TF publisher for sensor frames
        Node(
            package='static_tf_publisher',
            executable='static_tf_broadcaster',
            name='static_tf_broadcaster',
            output='screen'
        ),
    ])
```

**Test command:**
```bash
ros2 launch rover_launch phase_2_tf.launch.py
# In another terminal:
ros2 run tf2_tools view_frames.py
evince frames.pdf
```

---

## phase_3_ekf.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get rover_launch config path
    pkg_share = FindPackageShare('rover_launch')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'ekf_params.yaml'])
    
    return LaunchDescription([
        # Static TF (from previous phase)
        Node(
            package='static_tf_publisher',
            executable='static_tf_broadcaster',
            name='static_tf_broadcaster'
        ),
        
        # Robot localization EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[config_file],
            remappings=[('odometry/filtered', '/odom')],
            output='screen'
        ),
    ])
```

---

## phase_4_rtabmap.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('rover_launch')
    rtabmap_config = PathJoinSubstitution([pkg_share, 'config', 'rtabmap_params.yaml'])
    
    return LaunchDescription([
        # Sensors + TF + EKF (all previous phases included)
        
        # RTAB-Map SLAM
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            parameters=[
                {'frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'subscribe_depth': True},
                {'subscribe_odom_info': True},
                {'queue_size': 10},
                # Moon terrain parameters
                {'GFTT/MaxFeatures': '500'},  # Reduce for low texture
                {'GFTT/MinDistance': '5'},
                {'Vis/MinInliers': '3'},
                {'Vis/InlierDistance': '0.1'},
            ],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('depth/image', '/tof/depth/image_raw'),
                ('odom', '/odom'),
            ],
            output='screen'
        ),
    ])
```

---

## phase_5_terrain_layer.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('rover_launch')
    terrain_config = PathJoinSubstitution([pkg_share, 'config', 'terrain_layer_params.yaml'])
    
    return LaunchDescription([
        # Terrain/obstacle layer node (reads ToF points)
        Node(
            package='costmap_2d',
            executable='costmap_2d_node',
            name='terrain_layer',
            parameters=[terrain_config],
            output='screen'
        ),
    ])
```

---

## phase_6_auto_exposure.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Auto-exposure controller
        Node(
            package='auto_exposure_controller_node',
            executable='auto_exposure_controller',
            name='auto_exposure',
            parameters=[
                {'target_brightness': 128},
                {'brightness_tolerance': 10},
                {'max_exposure_time_ms': 30},
                {'min_exposure_time_ms': 1},
                {'adjustment_rate_hz': 2.0},
            ],
            output='screen'
        ),
    ])
```

---

## phase_7_perception.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # IMX500 classifier
        Node(
            package='imx500_classifier_node',
            executable='imx500_classifier_node',
            name='imx500_classifier',
            output='screen'
        ),
        
        # YOLO detector
        Node(
            package='yolo_detector_node',
            executable='yolo_detector_node',
            name='yolo_detector',
            parameters=[
                {'model_path': '/opt/yolo_models/landmark_detector.pt'},
                {'confidence_threshold': 0.5},
            ],
            output='screen'
        ),
        
        # Landmark map server
        Node(
            package='landmark_map_server',
            executable='landmark_map_server',
            name='landmark_map_server',
            output='screen'
        ),
        
        # Mission health monitor
        Node(
            package='mission_health_monitor',
            executable='mission_health_monitor',
            name='health_monitor',
            output='screen'
        ),
    ])
```

---

## phase_8_localization.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Landmark verification
        Node(
            package='landmark_verification_node',
            executable='landmark_verification_node',
            name='landmark_verification',
            output='screen'
        ),
        
        # Mission health monitor (continued)
        Node(
            package='mission_health_monitor',
            executable='mission_health_monitor',
            name='health_monitor',
            output='screen'
        ),
    ])
```

---

## phase_9_nav2.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('rover_launch')
    nav2_config = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    
    return LaunchDescription([
        # Nav2 lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[
                {'autostart': True},
                {'node_names': ['planner_server', 'controller_server', 'bt_navigator']},
            ],
            output='screen'
        ),
        
        # Nav2 planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_config],
            output='screen'
        ),
        
        # Nav2 controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_config],
            output='screen'
        ),
        
        # Nav2 BT navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_config],
            output='screen'
        ),
    ])
```

---

## phase_10_waypoint_recording.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Waypoint recorder
        Node(
            package='waypoint_recorder_node',
            executable='waypoint_recorder_node',
            name='waypoint_recorder',
            parameters=[
                {'recording': True},
                {'output_file': '/tmp/waypoints.yaml'},
                {'record_distance': 0.5},
                {'record_time': 5.0},
            ],
            output='screen'
        ),
        
        # Waypoint player
        Node(
            package='waypoint_player_node',
            executable='waypoint_player_node',
            name='waypoint_player',
            output='screen'
        ),
    ])
```

---

## phase_11_full_mission.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rover_launch_dir = FindPackageShare('rover_launch')
    
    return LaunchDescription([
        # Phase 1: Sensors (requires serial bridge running in separate terminal)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{rover_launch_dir}/launch/phase_1_sensors.launch.py'
            )
        ),
        
        # Phase 2: TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{rover_launch_dir}/launch/phase_2_tf.launch.py'
            )
        ),
        
        # Phase 3: EKF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{rover_launch_dir}/launch/phase_3_ekf.launch.py'
            )
        ),
        
        # Phase 4: RTAB-Map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{rover_launch_dir}/launch/phase_4_rtabmap.launch.py'
            )
        ),
        
        # Phase 5: Terrain layer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{rover_launch_dir}/launch/phase_5_terrain_layer.launch.py'
            )
        ),
        
        # Phase 6: Auto-exposure
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{rover_launch_dir}/launch/phase_6_auto_exposure.launch.py'
            )
        ),
        
        # Phase 7: Perception
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{rover_launch_dir}/launch/phase_7_perception.launch.py'
            )
        ),
        
        # Phase 8: Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{rover_launch_dir}/launch/phase_8_localization.launch.py'
            )
        ),
        
        # Phase 9: Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{rover_launch_dir}/launch/phase_9_nav2.launch.py'
            )
        ),
        
        # Phase 10: Waypoints
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{rover_launch_dir}/launch/phase_10_waypoint_recording.launch.py'
            )
        ),
        
        # Mission logger
        Node(
            package='mission_logger_node',
            executable='mission_logger_node',
            name='mission_logger',
            output='screen'
        ),
        
        # Streaming node for live feed
        Node(
            package='streaming_node',
            executable='streaming_node',
            name='streaming_server',
            parameters=[
                {'port': 8080},
                {'resolution_width': 320},
                {'resolution_height': 240},
            ],
            output='screen'
        ),
    ])
```

**LAUNCH FULL SYSTEM:**
```bash
# Terminal 1 - Start serial bridge FIRST:
ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

# Terminal 2 - Launch full mission:
source ~/ros2_moon_rover/install/setup.bash
ros2 launch rover_launch phase_11_full_mission.launch.py
```

---

**Next document: BUILD_PROCEDURES.md with exact step-by-step instructions**
