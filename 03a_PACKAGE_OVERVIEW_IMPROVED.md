# 📦 03a_PACKAGE_OVERVIEW.md - Complete Package Reference Guide
# Updated January 6, 2026 - All 23 Packages Organized by Function

---

## 🎯 PURPOSE OF THIS DOCUMENT

This is your **complete package reference guide** showing:
- All 23 ROS 2 packages in your system
- Where each package's code is located
- What each package does
- Which phase(s) each package appears in
- How to build and test each package

**Use this to find any package quickly during development.**

---

## 🗂️ PACKAGES ORGANIZED BY CATEGORY

### CATEGORY 1: CUSTOM MESSAGE DEFINITIONS (1 package)

#### 📋 rover_msgs
**Location:** `src/rover_msgs/`  
**Document:** 01_ROVER_MSGS.md  
**Purpose:** Defines 8 custom ROS 2 message types for inter-node communication  
**Phase:** 0 (prerequisite - build first!)  
**Build Priority:** ⭐⭐⭐ (CRITICAL - must build before any node)  

**Messages defined:**
- `Odometry2D` — 2D odometry with covariance
- `Classification` — AI classifier output (label, confidence, bbox)
- `LandmarkDetection` — 2D landmark detection
- `Landmark3D` — 3D landmark position in map frame
- `LandmarkMap` — Collection of detected landmarks
- `MissionHealth` — System health status (OK, DEGRADED, CRITICAL)
- `MissionEvent` — Mission event notifications
- `LandmarkVerification` — Landmark verification results

**Build:**
```bash
colcon build --packages-select rover_msgs --symlink-install
```

**Test:** None (verify in other packages that can import these messages)

---

### CATEGORY 2: HARDWARE DRIVERS (4 packages)

#### 🏎️ vex_driver_node
**Location:** `src/vex_driver_node/`  
**Document:** 02_VEX_DRIVER.md  
**Purpose:** Reads wheel encoders from VEX V5 brain over serial  
**Hardware:** VEX V5 brain (motor controller) via USB serial  
**Publishes:** `/vex/odom_raw` (nav_msgs/Odometry @ 50-100 Hz)  
**Phase:** 1 (Sensor Bring-Up)  
**Language:** C++  
**Lines of Code:** 430  
**Dependencies:** rclcpp, nav_msgs, serial communication  

**Build:**
```bash
colcon build --packages-select vex_driver_node --symlink-install
```

**Test:**
```bash
ros2 run vex_driver_node vex_driver_node
ros2 topic echo /vex/odom_raw  # Should see encoder readings
```

**Tune:** `config/vex_params.yaml`
- `serial_port` — Serial device (default: /dev/ttyUSB0)
- `baud_rate` — Serial speed (default: 115200)
- `wheel_radius_m` — Calibrated wheel radius
- `wheel_separation_m` — Distance between left/right wheels

---

#### 🧭 imu_driver_node
**Location:** `src/imu_driver_node/`  
**Document:** 03_IMU_DRIVER.md  
**Purpose:** Reads accelerometer & gyroscope from MPU6050 via I2C  
**Hardware:** MPU6050 IMU on I2C bus (address 0x68)  
**Publishes:** `/imu/data` (sensor_msgs/Imu @ 100 Hz)  
**Phase:** 1 (Sensor Bring-Up)  
**Language:** C++  
**Lines of Code:** 280  
**Dependencies:** rclcpp, sensor_msgs, i2cdev  

**Build:**
```bash
colcon build --packages-select imu_driver_node --symlink-install
```

**Test:**
```bash
ros2 run imu_driver_node imu_driver_node
ros2 topic echo /imu/data  # Should see IMU data at 100 Hz
# Z-acceleration should be ~-9.81 m/s^2 (gravity) when stationary
```

**Tune:** `config/imu_calibration.yaml`
- `accelerometer_offsets` — X, Y, Z offset (m/s^2)
- `gyroscope_offsets` — X, Y, Z offset (rad/s)

---

#### 📷 imx500_camera_node
**Location:** `src/imx500_camera_node/`  
**Document:** 04_CAMERA_AND_TOF_DRIVERS.md  
**Purpose:** Streams RGB video from Pi AI Camera (IMX500)  
**Hardware:** Pi AI Camera with IMX500 (CSI connector)  
**Publishes:** 
- `/camera/image_raw` (sensor_msgs/Image @ 10-30 Hz)
- `/camera/camera_info` (sensor_msgs/CameraInfo)
**Phase:** 1 (Sensor Bring-Up)  
**Language:** Python  
**Lines of Code:** 200+  
**Dependencies:** rclpy, sensor_msgs, cv_bridge, libcamera  

**Build:**
```bash
colcon build --packages-select imx500_camera_node --symlink-install
```

**Test:**
```bash
ros2 run imx500_camera_node imx500_camera_node
ros2 topic echo /camera/camera_info  # Should see intrinsics
rqt_image_view  # Subscribe to /camera/image_raw
```

**Tune:** `config/camera_params.yaml`
- `resolution` — Image resolution (default: 1280x720)
- `frame_rate` — FPS (default: 30)
- `exposure_time_ms` — Exposure (default: 33ms)
- `analogue_gain` — Sensor gain (default: 1.0)

---

#### 🔍 tof_camera_node
**Location:** `src/tof_camera_node/`  
**Document:** 04_CAMERA_AND_TOF_DRIVERS.md  
**Purpose:** Streams depth data from Arducam ToF camera  
**Hardware:** Arducam ToF camera (CSI connector)  
**Publishes:**
- `/tof/depth/image_raw` (sensor_msgs/Image @ 10-20 Hz)
- `/tof/points` (sensor_msgs/PointCloud2)
**Phase:** 1 (Sensor Bring-Up)  
**Language:** Python  
**Lines of Code:** 250+  
**Dependencies:** rclpy, sensor_msgs, cv_bridge  

**Build:**
```bash
colcon build --packages-select tof_camera_node --symlink-install
```

**Test:**
```bash
ros2 run tof_camera_node tof_camera_node
ros2 topic echo /tof/points  # Should see 3D point cloud
```

**Tune:** `config/tof_params.yaml`
- `depth_range_m` — Max depth to capture (default: 4.0m)
- `frame_rate` — FPS (default: 15)
- `point_cloud_scale` — Decimation factor (default: 4)

---

### CATEGORY 3: TRANSFORM PUBLISHER (1 package)

#### 🌍 static_tf_publisher
**Location:** `src/static_tf_publisher/`  
**Document:** 05_STATIC_TF_PUBLISHER.md  
**Purpose:** Broadcasts static TF tree (base_link → sensor frames)  
**Publishes:** Static transforms via TF2  
**Phase:** 2 (TF Setup)  
**Language:** C++  
**Lines of Code:** 150  
**Dependencies:** tf2_ros, geometry_msgs  

**Build:**
```bash
colcon build --packages-select static_tf_publisher --symlink-install
```

**Test:**
```bash
ros2 run static_tf_publisher static_tf_broadcaster
# In another terminal:
ros2 run tf2_tools view_frames.py
# Should show: map → odom → base_link → {camera, tof, imu}
```

**Configure:** `config/static_transforms.yaml`
- `base_link → camera_link` — +20cm forward, +10cm up
- `base_link → tof_link` — +20cm forward, +5cm right, +10cm up
- `base_link → imu_link` — center, +5cm up

---

### CATEGORY 4: LOCALIZATION & FUSION (1 custom + 2 source packages)

#### ⚙️ ekf_fusion_node (CUSTOM - NEW!)
**Location:** `src/ekf_fusion_node/`  
**Document:** 06_EKF_FUSION_NODE.md  
**Purpose:** Fuses wheel odometry + IMU using Extended Kalman Filter  
**Inputs:** 
- `/vex/odom_raw` (wheel encoders)
- `/imu/data` (accelerometer + gyro)
**Outputs:** 
- `/odom` (nav_msgs/Odometry @ 50 Hz)
- TF: `odom → base_link`
**Phase:** 3 (EKF Odometry Fusion)  
**Language:** C++ (wrapper), Python config  
**Lines of Code:** 300+  
**Uses:** robot_localization ekf_node  
**Config:** `config/ekf_params.yaml` (200+ lines, extensively documented)  

**Build:**
```bash
# Must clone robot_localization first
colcon build --packages-select ekf_fusion_node --symlink-install
```

**Test:**
```bash
# Start sensors + TF first
ros2 launch rover_launch phase_1_sensors.launch.py &
ros2 launch ekf_fusion_node ekf.launch.py

# In another terminal:
ros2 topic echo /odom  # Should see position updating
# Drive rover in square, check that position returns close to origin
```

**Tune:** `config/ekf_params.yaml`
- **Process noise** — How much to trust motion model (0.03 = good for moon)
- **Measurement noise** — How much to trust sensors (0.05 = good)
- **Initial covariance** — Starting uncertainty (1.0 = 1m uncertainty)
- **See extensive comments in file for tuning guide**

---

#### 📍 robot_localization (SOURCE PACKAGE)
**Source:** `https://github.com/cra-ros-pkg/robot_localization.git`  
**Purpose:** Provides ekf_node and ukf_node  
**Build:** Included in Phase 0 setup  
**Note:** ekf_fusion_node wraps this package

---

#### 🗺️ rtabmap_ros (SOURCE PACKAGE)
**Source:** `https://github.com/ros-perception/rtabmap_ros.git`  
**Purpose:** Visual SLAM integration with ROS 2  
**Launches in:** Phase 4 (RTAB-Map SLAM)  
**Build:** Included in Phase 0 setup  

---

### CATEGORY 5: PERCEPTION PIPELINE (3 packages - NEW!)

#### 🧠 imx500_classifier_node
**Location:** `src/imx500_classifier_node/`  
**Document:** 07_PERCEPTION_NODES.md (Package 1)  
**Purpose:** On-device AI classification using NPU hardware  
**Inputs:** `/camera/image_raw`  
**Outputs:** `/ai_camera/classification` (custom Classification message)  
**Phase:** 7 (Perception Pipeline)  
**Language:** Python  
**Lines of Code:** 200+  
**Classes:** terrain, rock, crater, rover, landmark (configurable)  

**Build:**
```bash
colcon build --packages-select imx500_classifier_node --symlink-install
```

**Test:**
```bash
ros2 run imx500_classifier_node imx500_classifier_node
ros2 topic echo /ai_camera/classification
# Should see classifications at ~10 Hz
```

**Configure:** Parameters in node code
- `model_path` — Path to TFLite model
- `confidence_threshold` — Min confidence (0.6)
- `classes` — List of class names
- `enable_npu` — Use hardware NPU acceleration

---

#### ☀️ auto_exposure_controller_node
**Location:** `src/auto_exposure_controller_node/`  
**Document:** 07_PERCEPTION_NODES.md (Package 2)  
**Purpose:** Dynamic exposure control for moon lighting variations  
**Inputs:** 
- `/camera/image_raw`
- `/imu/data` (for motion detection)
- `/ai_camera/classification` (optional feedback)
**Outputs:** Camera parameter updates (exposure, gain, ROI)  
**Phase:** 6 (Auto-Exposure Controller)  
**Language:** Python  
**Lines of Code:** 270+  

**Build:**
```bash
colcon build --packages-select auto_exposure_controller_node --symlink-install
```

**Test:**
```bash
ros2 run auto_exposure_controller_node auto_exposure_controller_node
# Observe video brightness stabilizing over time
```

**Configure:** `config/auto_exposure_params.yaml`
- `target_brightness` — Target image brightness (128)
- `max_exposure_time_ms` — Max exposure (33ms = 30fps)
- `max_gain` — Max analog gain (4.0x)
- `adjustment_rate_hz` — Update frequency (2.0 Hz)

---

#### 🎯 yolo_detector_node
**Location:** `src/yolo_detector_node/`  
**Document:** 07_PERCEPTION_NODES.md (Package 3)  
**Purpose:** Landmark detection using YOLO framework  
**Inputs:** `/camera/image_raw`  
**Outputs:** `/landmarks/detections` (custom LandmarkDetection messages)  
**Phase:** 7 (Perception Pipeline)  
**Language:** Python  
**Framework:** YOLO (any version - can be downloaded)  

**Build:**
```bash
colcon build --packages-select yolo_detector_node --symlink-install
```

**Test:**
```bash
ros2 run yolo_detector_node yolo_detector_node
ros2 topic echo /landmarks/detections
```

**Configure:** Parameters in node
- `model_path` — YOLO model file
- `confidence_threshold` — Min detection confidence
- `classes` — Landmark class names

---

### CATEGORY 6: LOCALIZATION & MISSION MONITORING (3 packages - NEW!)

#### 🗂️ landmark_map_server
**Location:** `src/landmark_map_server/`  
**Document:** 08_LOCALIZATION_NODES.md (Package 1)  
**Purpose:** Persistent landmark database with save/load  
**Inputs:** `/landmarks/positions` (3D detections)  
**Outputs:** `/landmarks/map` (landmark collection)  
**Services:** save_landmarks, load_landmarks, clear_landmarks  
**Phase:** 8 (Localization Nodes)  
**Language:** Python  
**Lines of Code:** 300+  
**Storage:** YAML or JSON files  

**Build:**
```bash
colcon build --packages-select landmark_map_server --symlink-install
```

**Test:**
```bash
ros2 run landmark_map_server landmark_map_server
# Publish some landmarks to /landmarks/positions
ros2 service call save_landmarks std_srvs/Trigger
# Check /tmp/landmark_map.yaml was created
```

**Configure:** Parameters in node
- `map_file` — Save location (default: /tmp/landmark_map.yaml)
- `duplicate_distance_threshold` — Merge nearby detections (0.3m)

---

#### 💚 mission_health_monitor
**Location:** `src/mission_health_monitor/`  
**Document:** 08_LOCALIZATION_NODES.md (Package 2)  
**Purpose:** System health tracking and diagnostics  
**Inputs:**
- `/odom` (odometry covariance)
- `/imu/data` (acceleration anomalies)
- RTAB-Map stats
**Outputs:**
- `/mission_health` (MissionHealth message)
- `/mission_events` (MissionEvent messages)
**Phase:** 8 (Localization Nodes)  
**Language:** Python  
**Lines of Code:** 250+  
**Status Levels:** OK, DEGRADED, CRITICAL  

**Build:**
```bash
colcon build --packages-select mission_health_monitor --symlink-install
```

**Test:**
```bash
ros2 run mission_health_monitor mission_health_monitor
ros2 topic echo /mission_health  # Monitor system health
```

**Configure:** Thresholds in node
- `covariance_trace_critical` — Max acceptable covariance (2.0)
- `feature_count_critical` — Min SLAM features (5)
- `loop_closure_rate_critical` — Min loop closure rate (0.0)

---

#### ✅ landmark_verification_node
**Location:** `src/landmark_verification_node/`  
**Document:** 08_LOCALIZATION_NODES.md (Package 3)  
**Purpose:** Cross-rover landmark verification for localization validation  
**Inputs:**
- `/landmarks/positions` (observed landmarks)
- `/landmarks/map` (landmark reference map)
**Outputs:** `/landmarks/verification` (LandmarkVerification messages)  
**Phase:** 8 (Localization Nodes)  
**Language:** Python  
**Lines of Code:** 200+  

**Build:**
```bash
colcon build --packages-select landmark_verification_node --symlink-install
```

**Test:**
```bash
ros2 run landmark_verification_node landmark_verification_node
ros2 topic echo /landmarks/verification  # See verification results
```

**Configure:** Error thresholds
- `error_ok_threshold` — OK if error < 0.3m
- `error_degraded_threshold` — DEGRADED if 0.3-1.0m
- Report CRITICAL if error > 1.0m

---

### CATEGORY 7: NAVIGATION & MISSION SUPPORT (4 packages - NEW!)

#### 🎙️ waypoint_recorder_node
**Location:** `src/waypoint_recorder_node/`  
**Document:** 09_NAVIGATION_NODES.md (Package 1)  
**Purpose:** Records rover trajectory for mission replay  
**Inputs:** TF (map → base_link)  
**Outputs:** 
- `/recorded_waypoints` (geometry_msgs/PoseStamped array)
- Saves to YAML/JSON file
**Services:** start_recording, save_waypoints, clear_waypoints  
**Phase:** 10 (Waypoint Recording)  
**Language:** Python  
**Lines of Code:** 250+  

**Build:**
```bash
colcon build --packages-select waypoint_recorder_node --symlink-install
```

**Test:**
```bash
ros2 run waypoint_recorder_node waypoint_recorder_node
ros2 service call start_recording std_srvs/SetBool "data: true"
# Drive rover around
ros2 service call save_waypoints std_srvs/Trigger
# Check /tmp/waypoints.yaml
```

**Configure:** Parameters in node
- `output_file` — Save location
- `record_distance` — Distance between waypoints (0.5m)
- `record_time` — Time between waypoints (5s)

---

#### ▶️ waypoint_player_node
**Location:** `src/waypoint_player_node/`  
**Document:** 09_NAVIGATION_NODES.md (Package 2)  
**Purpose:** Replays recorded trajectories using Nav2  
**Inputs:** Waypoint file (YAML/JSON)  
**Outputs:** Nav2 goal commands  
**Services:** start_replay, stop_replay  
**Phase:** 10 (Waypoint Recording)  
**Language:** Python  
**Lines of Code:** 200+  

**Build:**
```bash
colcon build --packages-select waypoint_player_node --symlink-install
```

**Test:**
```bash
ros2 run waypoint_player_node waypoint_player_node
ros2 service call start_replay std_srvs/Trigger
# Rover should follow recorded path
```

---

#### 📡 mission_logger_node
**Location:** `src/mission_logger_node/`  
**Document:** 09_NAVIGATION_NODES.md (Package 3)  
**Purpose:** Records all sensor data to rosbag for post-analysis  
**Inputs:** All topics (configurable list)  
**Outputs:** Rosbag2 archive  
**Phase:** 11 (Full Mission Integration)  
**Language:** Python  
**Lines of Code:** 150+  

**Build:**
```bash
colcon build --packages-select mission_logger_node --symlink-install
```

**Test:**
```bash
ros2 run mission_logger_node mission_logger_node
# Check /tmp/rover_missions/ for recorded bags
```

**Configure:** Topics to record
- `/camera/image_raw`
- `/tof/depth/image_raw`
- `/odom`
- `/imu/data`
- `/mission_health`
- `/landmarks/positions`

---

#### 📹 streaming_node
**Location:** `src/streaming_node/`  
**Document:** 09_NAVIGATION_NODES.md (Package 4)  
**Purpose:** Live MJPEG video streaming over HTTP  
**Inputs:** `/camera/image_raw`  
**Outputs:** HTTP MJPEG stream at http://<pi-ip>:8080/stream  
**Phase:** 11 (Full Mission Integration)  
**Language:** Python  
**Lines of Code:** 200+  

**Build:**
```bash
colcon build --packages-select streaming_node --symlink-install
```

**Test:**
```bash
ros2 run streaming_node streaming_node
# Open browser to http://localhost:8080/stream
```

**Configure:** Parameters in node
- `port` — HTTP server port (8080)
- `resolution_width` — Stream width (320)
- `resolution_height` — Stream height (240)
- `quality` — JPEG quality (80)

---

### CATEGORY 8: LAUNCH SYSTEM (1 package)

#### 🚀 rover_launch
**Location:** `src/rover_launch/`  
**Document:** 06_LAUNCH_FILES.md  
**Purpose:** All 11 phased launch files + configuration files  
**Launch Files:**
1. phase_0_workspace_setup.launch.py
2. phase_1_sensors.launch.py
3. phase_2_tf.launch.py
4. phase_3_ekf.launch.py
5. phase_4_rtabmap.launch.py
6. phase_5_terrain_layer.launch.py
7. phase_6_auto_exposure.launch.py
8. phase_7_perception.launch.py
9. phase_8_nav2_static.launch.py
10. phase_9_nav2_rtabmap.launch.py
11. phase_10_waypoint_recording.launch.py
12. phase_11_full_mission.launch.py

**Configuration Files:**
- `config/rtabmap_params.yaml` (SLAM tuning)
- `config/nav2_params.yaml` (navigation tuning)
- `config/ekf_params.yaml` (fusion tuning)
- `config/static_transforms.yaml` (TF tree)
- `config/imx500_intrinsics.yaml` (camera calibration)
- `config/tof_intrinsics.yaml` (depth calibration)
- Additional sensor configs

**Use:** Launch each phase sequentially during bring-up

---

### CATEGORY 9: SOURCE PACKAGES (3 packages)

#### 🔄 navigation2
**Source:** `https://github.com/ros-planning/navigation2.git`  
**Purpose:** Complete navigation stack with Nav2 nodes  
**Launches in:** Phase 8-11  
**Build:** Phase 0 setup  

---

#### 🎥 image_common
**Source:** `https://github.com/ros-perception/image_common.git`  
**Purpose:** Camera utilities and CameraInfo definitions  
**Launches in:** Phase 1  
**Build:** Phase 0 setup  

---

#### 📊 diagnostics
**Source:** `https://github.com/ros-perception/diagnostics.git`  
**Purpose:** System diagnostics and monitoring  
**Launches in:** Phase 8+  
**Build:** Phase 0 setup  

---

## 🔍 QUICK LOOKUP TABLE

| Package | Type | Phase | Language | Lines | Status |
|---------|------|-------|----------|-------|--------|
| rover_msgs | Messages | 0 | YAML | 200 | ✅ |
| vex_driver_node | Driver | 1 | C++ | 430 | ✅ |
| imu_driver_node | Driver | 1 | C++ | 280 | ✅ |
| imx500_camera_node | Driver | 1 | Python | 200+ | ✅ |
| tof_camera_node | Driver | 1 | Python | 250+ | ✅ |
| static_tf_publisher | Transform | 2 | C++ | 150 | ✅ |
| ekf_fusion_node | Fusion | 3 | C++/YAML | 300+ | ✅ NEW |
| imx500_classifier_node | Perception | 7 | Python | 200+ | ✅ NEW |
| auto_exposure_controller_node | Perception | 6 | Python | 270+ | ✅ NEW |
| yolo_detector_node | Perception | 7 | Python | — | ✅ NEW |
| landmark_map_server | Localization | 8 | Python | 300+ | ✅ NEW |
| mission_health_monitor | Monitoring | 8 | Python | 250+ | ✅ NEW |
| landmark_verification_node | Localization | 8 | Python | 200+ | ✅ NEW |
| waypoint_recorder_node | Navigation | 10 | Python | 250+ | ✅ NEW |
| waypoint_player_node | Navigation | 10 | Python | 200+ | ✅ NEW |
| mission_logger_node | Logging | 11 | Python | 150+ | ✅ NEW |
| streaming_node | Streaming | 11 | Python | 200+ | ✅ NEW |
| rover_launch | Launch | All | Python | 300+ | ✅ |
| robot_localization | Source | 3 | — | — | ✅ |
| rtabmap_ros | Source | 4 | — | — | ✅ |
| navigation2 | Source | 8+ | — | — | ✅ |
| image_common | Source | 1 | — | — | ✅ |
| diagnostics | Source | 8+ | — | — | ✅ |

**TOTAL: 23 packages, 5,000+ lines of code, 100% complete** ✅

---

## 🚀 BUILD ORDER (CRITICAL!)

**You MUST follow this order:**

```
Phase 0: Setup Workspace
  ↓
Phase 1: Build rover_msgs (FIRST!)
  ↓
Phase 2: Build source packages (rtabmap, nav2, etc.)
  ↓
Phase 3: Build drivers (vex, imu, cameras, tf)
  ↓
Phase 4: Build fusion (ekf)
  ↓
Phase 5: Build perception (classifiers, detectors)
  ↓
Phase 6: Build localization (landmark, health, verification)
  ↓
Phase 7: Build navigation (waypoints, logging, streaming)
```

**Command:**
```bash
cd ~/ros2_moon_rover
colcon build --packages-select rover_msgs --symlink-install
colcon build --symlink-install  # Build all remaining
```

---

## 🧪 TEST ORDER

**Test each phase sequentially:**

1. Phase 1: `ros2 launch rover_launch phase_1_sensors.launch.py`
2. Phase 2: `ros2 launch rover_launch phase_2_tf.launch.py`
3. Phase 3: `ros2 launch rover_launch phase_3_ekf.launch.py`
4. ... (continue through Phase 11)

**Never skip ahead — validate each phase before proceeding.**

---

## 📍 WHERE TO FIND CODE FOR EACH PACKAGE

| Package | Code Location | Document |
|---------|---------------|----------|
| rover_msgs | `src/rover_msgs/` | 01_ROVER_MSGS.md |
| vex_driver_node | `src/vex_driver_node/` | 02_VEX_DRIVER.md |
| imu_driver_node | `src/imu_driver_node/` | 03_IMU_DRIVER.md |
| imx500_camera_node | `src/imx500_camera_node/` | 04_CAMERA_AND_TOF_DRIVERS.md |
| tof_camera_node | `src/tof_camera_node/` | 04_CAMERA_AND_TOF_DRIVERS.md |
| static_tf_publisher | `src/static_tf_publisher/` | 05_STATIC_TF_PUBLISHER.md |
| ekf_fusion_node | `src/ekf_fusion_node/` | 06_EKF_FUSION_NODE.md |
| imx500_classifier_node | `src/imx500_classifier_node/` | 07_PERCEPTION_NODES.md |
| auto_exposure_controller_node | `src/auto_exposure_controller_node/` | 07_PERCEPTION_NODES.md |
| yolo_detector_node | `src/yolo_detector_node/` | 07_PERCEPTION_NODES.md |
| landmark_map_server | `src/landmark_map_server/` | 08_LOCALIZATION_NODES.md |
| mission_health_monitor | `src/mission_health_monitor/` | 08_LOCALIZATION_NODES.md |
| landmark_verification_node | `src/landmark_verification_node/` | 08_LOCALIZATION_NODES.md |
| waypoint_recorder_node | `src/waypoint_recorder_node/` | 09_NAVIGATION_NODES.md |
| waypoint_player_node | `src/waypoint_player_node/` | 09_NAVIGATION_NODES.md |
| mission_logger_node | `src/mission_logger_node/` | 09_NAVIGATION_NODES.md |
| streaming_node | `src/streaming_node/` | 09_NAVIGATION_NODES.md |
| rover_launch | `src/rover_launch/` | 06_LAUNCH_FILES.md |

---

## ✅ VERIFICATION CHECKLIST

- [ ] All 23 packages accounted for
- [ ] Understand which phase each package belongs to
- [ ] Know where to find code for each package
- [ ] Understand build order (rover_msgs first!)
- [ ] Ready to proceed with Phase 0 setup

---

**🎉 You now have a complete package reference guide!**

**Next: Read 07_BUILD_PROCEDURES.md to start building**
