# ROS 2 Humble SLAM + Navigation System - Implementation Roadmap

## 📋 PROJECT STRUCTURE

```
ros2_moon_rover/
├── src/
│   ├── rover_hardware/           # Hardware interface packages
│   │   ├── vex_driver_node/
│   │   ├── imu_driver_node/
│   │   ├── imx500_camera_node/
│   │   ├── tof_camera_node/
│   │   └── static_tf_publisher/
│   │
│   ├── rover_perception/         # Vision & AI
│   │   ├── imx500_classifier_node/
│   │   ├── yolo_detector_node/
│   │   ├── landmark_localizer_node/
│   │   └── auto_exposure_controller_node/
│   │
│   ├── rover_localization/       # SLAM & Odometry
│   │   ├── ekf_fusion_node/
│   │   ├── rtabmap_integration/
│   │   └── landmark_map_server/
│   │
│   ├── rover_navigation/         # Nav2 & Terrain
│   │   ├── terrain_layer_plugin/
│   │   ├── nav2_integration/
│   │   ├── waypoint_recorder_node/
│   │   └── waypoint_player_node/
│   │
│   ├── rover_monitoring/         # Health & Logging
│   │   ├── mission_health_monitor/
│   │   ├── landmark_verification_node/
│   │   ├── streaming_node/
│   │   └── mission_logger_node/
│   │
│   ├── rover_msgs/               # Custom messages
│   │   ├── msg/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── rover_launch/             # Unified launch files
│       ├── phase_0_workspace_setup.launch.py
│       ├── phase_1_sensors.launch.py
│       ├── phase_2_tf.launch.py
│       ├── ... (phases 3-11)
│       └── full_mission.launch.py
│
├── install/
├── build/
├── log/
│
├── colcon_workspace_setup.sh     # Workspace initialization
└── BUILD_INSTRUCTIONS.md         # Step-by-step build guide
```

---

## 🎯 PHASED BUILD PLAN

### **Phase 0: Workspace Setup (Source-Only)**
- [ ] Create ROS 2 Humble workspace
- [ ] Clone all required source packages:
  - `ros2/ros2` core packages
  - `ros-perception/rtabmap_ros`
  - `ros-planning/navigation2`
  - `ros-planning/navigation2_msgs`
  - `boilerplate/image_common`
  - `boilerplate/geometry2`
  - `boilerplate/rclcpp`, `rclpy`
- [ ] Create custom message package: `rover_msgs`
- [ ] Initialize all custom node packages
- **Definition of Done**: `colcon build` succeeds with zero source packages

### **Phase 1: Sensor Bring-Up (Independent drivers)**
- [ ] VEX serial driver → `/vex/odom_raw`
- [ ] IMU driver → `/imu/data`
- [ ] IMX500 RGB camera → `/camera/image_raw`, `/camera/camera_info`
- [ ] ToF camera → `/tof/depth/image_raw`, `/tof/points`
- **Definition of Done**: All 4 sensors publishing data verified in `ros2 topic list` + sample bag file

### **Phase 2: TF Setup**
- [ ] Static TF publisher for:
  - `base_link → camera_link`
  - `base_link → tof_link`
  - `base_link → imu_link`
- [ ] Verify TF tree in RViz: `map → odom → base_link → {camera, tof, imu}`
- **Definition of Done**: `ros2 run tf2_tools view_frames.py` shows correct tree

### **Phase 3: EKF Fusion**
- [ ] EKF node fuses `/vex/odom_raw` + `/imu/data`
- [ ] Outputs `/odom` (nav_msgs/Odometry)
- [ ] Publishes `odom → base_link` TF
- [ ] Tune process/measurement noise
- **Definition of Done**: Odometry drifts <0.5 m/min, covariance values reasonable

### **Phase 4: RTAB-Map (Camera + Odom)**
- [ ] Build `rtabmap_ros` from source
- [ ] RTAB-Map node reads `/camera/image_raw`, `/camera/camera_info`, `/odom`
- [ ] Publishes `map → odom` TF, `/rtabmap/mapData`, `/rtabmap/map`
- [ ] Tune feature detection for low-texture terrain
- **Definition of Done**: Loop closure detection works indoors, map quality acceptable

### **Phase 5: Add ToF + Terrain Layer**
- [ ] Terrain layer plugin reads `/tof/points`
- [ ] Publishes costmap with traversability
- [ ] RTAB-Map ingests depth via ToF
- **Definition of Done**: Costmap updates at 10 Hz, RViz shows obstacles

### **Phase 6: Auto-Exposure Controller**
- [ ] Controller reads `/camera/image_raw`, `/imu/data`
- [ ] Adjusts camera parameters dynamically
- [ ] Publish parameter updates via `/camera_control/set_parameters`
- **Definition of Done**: Image brightness stable across lighting changes

### **Phase 7: Classification + Mission Health**
- [ ] IMX500 classifier node → `/ai_camera/classification`
- [ ] YOLO detector → `/landmarks/detections`
- [ ] Mission health monitor → `/mission_health`, `/mission_events`
- [ ] Landmark localizer → `/landmarks/positions`
- **Definition of Done**: Detections and classifications on live feed

### **Phase 8: Nav2 with Static Map**
- [ ] Build `navigation2` from source
- [ ] Nav2 stack reads static `/map` (from RViz or premade)
- [ ] Goal navigation works
- **Definition of Done**: Rover navigates to goal in known map

### **Phase 9: Nav2 + RTAB-Map Map**
- [ ] Nav2 ingests `/map` from RTAB-Map (dynamic)
- [ ] Waypoint following (goal_checker, waypoint_follower)
- **Definition of Done**: Rover follows waypoints dynamically

### **Phase 10: Waypoint Recording + Replay**
- [ ] Waypoint recorder saves poses
- [ ] Waypoint player replays with error checking
- [ ] Mission logging records all topics
- **Definition of Done**: Recorded and replayed trajectory matches

### **Phase 11: Full Mission + Logging + Replay**
- [ ] All subsystems integrated
- [ ] Mission logger records everything
- [ ] Landmark verification cross-checks
- [ ] Streaming node serves live feed
- **Definition of Done**: Full autonomous mission executes end-to-end

---

## 📦 SOURCE PACKAGES TO BUILD

### **ROS 2 Core (Required from apt)**
These are installed via `apt install ros-humble-*`:
- `ros-humble-ros-core` (already in system)
- `ros-humble-rclcpp`, `rclpy`
- `ros-humble-geometry2` (TF2)
- `ros-humble-sensor-msgs`
- `ros-humble-nav-msgs`
- `ros-humble-tf2-ros`
- `ros-humble-cv-bridge`
- `ros-humble-image-transport`

### **SOURCE PACKAGES (Clone to src/)**
```bash
cd ~/ros2_moon_rover/src

# SLAM
git clone --branch humble https://github.com/ros-perception/rtabmap_ros.git
git clone --branch humble https://github.com/introlab/rtabmap.git

# Navigation
git clone --branch humble https://github.com/ros-planning/navigation2.git

# Image transport & utilities
git clone --branch humble https://github.com/ros-perception/image_common.git

# Custom packages (yours - generated below)
mkdir -p rover_hardware rover_perception rover_localization rover_navigation rover_monitoring rover_msgs rover_launch
```

---

## 🔧 BUILD INSTRUCTIONS

### **Step 1: Create Workspace**
```bash
mkdir -p ~/ros2_moon_rover/src
cd ~/ros2_moon_rover
```

### **Step 2: Clone Source Packages**
See section above.

### **Step 3: Copy Generated Packages**
Copy all generated ROS 2 packages (from sections below) into `src/`.

### **Step 4: Install Dependencies**
```bash
cd ~/ros2_moon_rover
rosdep install --from-paths src --ignore-src -r -y
```

### **Step 5: Build Workspace**
```bash
colcon build --packages-select rover_msgs  # Build messages first
colcon build --symlink-install             # Build all
```

### **Step 6: Source Setup**
```bash
source ~/ros2_moon_rover/install/setup.bash
```

---

## 🚀 QUICK START

### **Launch Phase 1 (Sensors Only)**
```bash
source ~/ros2_moon_rover/install/setup.bash
ros2 launch rover_launch phase_1_sensors.launch.py
```

### **Verify Sensors**
```bash
# Terminal 2
ros2 topic list
ros2 topic echo /vex/odom_raw
ros2 topic echo /imu/data
ros2 topic echo /camera/image_raw
ros2 topic echo /tof/points
```

### **Full System (Phase 11)**
```bash
ros2 launch rover_launch full_mission.launch.py
```

---

## 📝 FILES GENERATED BELOW

This roadmap covers:

1. **Custom Message Definitions** (`rover_msgs/`)
2. **Hardware Drivers** (VEX, IMU, cameras)
3. **Sensor Integrations** (cameras → topics)
4. **Localization** (EKF, RTAB-Map integration)
5. **Perception** (classifiers, detectors, localizers)
6. **Navigation** (terrain layer, waypoint recorder)
7. **Monitoring** (health monitor, logging)
8. **Calibration Pipeline**
9. **All Launch Files** (11 phases + full mission)
10. **Configuration YAML Files**

---

## ⚙️ KEY CONFIGURATION PARAMETERS

See detailed YAML configs in respective sections:
- `ekf_params.yaml` — Sensor noise tuning
- `rtabmap_params.yaml` — Feature detection, loop closure
- `nav2_params.yaml` — Planner, controller settings
- `camera_params.yaml` — Exposure, ROI, gain
- `terrain_layer_params.yaml` — Costmap obstacles

---

## 🎓 DEBUGGING TIPS

- **TF errors**: `ros2 run tf2_tools view_frames.py && evince frames.pdf`
- **Topic not found**: `ros2 topic list` and verify node launched
- **Build errors**: Check `colcon build --symlink-install 2>&1 | tail -50`
- **RViz crashes**: Start with simple config, add layers incrementally
- **Low FPS on Pi**: Reduce image resolution, increase node periods

---

## 📞 NEXT STEPS

1. Read **Phase 0 Setup Instructions** (below)
2. Create workspace and clone source packages
3. Build **Phase 1 (Sensors)** independently
4. Test each sensor with `ros2 topic echo`
5. Proceed through phases sequentially
6. Tune parameters as you go

---

**Expected timeline: 2-3 weeks for full integration with testing**

Generated files start below. Each section is self-contained and can be built independently.
