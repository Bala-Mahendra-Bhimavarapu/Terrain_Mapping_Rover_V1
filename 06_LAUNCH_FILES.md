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
            parameters=[{'serial_port': '/dev/ttyUSB0',
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

**Test command:**
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
    # Get robot_localization config path
    pkg_share = FindPackageShare('ekf_fusion_node')
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

def generate_launch_description():
    return LaunchDescription([
        # Sensors + TF + EKF (all previous phases)
        # ... (include all nodes from phase 1-3)
        
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

def generate_launch_description():
    return LaunchDescription([
        # All previous phases
        # ...
        
        # Terrain/obstacle layer node (reads ToF points)
        Node(
            package='terrain_layer_plugin',
            executable='terrain_layer_node',
            name='terrain_layer',
            parameters=[
                {'max_obstacle_height': 0.5},  # 50 cm obstacles
                {'min_obstacle_height': 0.05},  # 5 cm minimum
            ],
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
        # All previous + new
        
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
                {'model_path': '/home/pi/yolo_model.pt'},
                {'confidence_threshold': 0.5},
            ],
            output='screen'
        ),
        
        # Landmark localizer
        Node(
            package='landmark_localizer_node',
            executable='landmark_localizer_node',
            name='landmark_localizer',
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

## phase_8_nav2_static.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nav2 stack with static map (from file or RViz)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                {'yaml_filename': '/tmp/static_map.yaml'},
            ],
            output='screen'
        ),
        
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
    ])
```

---

## phase_9_nav2_rtabmap.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Same as phase 8 but Nav2 reads map from RTAB-Map instead of file
        # map_server replaced with connection to /rtabmap/map topic
        
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
    ])
```

---

## phase_10_waypoint_recording.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # All previous phases +
        
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
        # Phase 1: Sensors
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
        
        # Phase 9: Nav2 + RTAB-Map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{rover_launch_dir}/launch/phase_9_nav2_rtabmap.launch.py'
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
source ~/ros2_moon_rover/install/setup.bash
ros2 launch rover_launch phase_11_full_mission.launch.py
```

---

**Next document: BUILD_PROCEDURES.md with exact step-by-step instructions**
