# ROS 2 Moon Rover: Complete Package Implementation Guide
# This document contains all remaining core packages in a consolidated, copy-paste format

## INDEX OF ALL PACKAGES

This guide covers:
1. ✅ rover_msgs (custom messages) - ALREADY CREATED
2. ✅ vex_driver_node - ALREADY CREATED
3. ✅ imu_driver_node - ALREADY CREATED
4. 📋 imx500_camera_node
5. 📋 tof_camera_node
6. 📋 static_tf_publisher
7. 📋 ekf_fusion_node
8. 📋 auto_exposure_controller_node
9. 📋 imx500_classifier_node
10. 📋 yolo_detector_node
11. 📋 landmark_localizer_node
12. 📋 landmark_map_server
13. 📋 mission_health_monitor_node
14. 📋 waypoint_recorder_node
15. 📋 terrain_layer_plugin
16. 📋 nav2_integration
17. 📋 streaming_node
18. 📋 All Phase Launch Files

---

# NEXT DOCUMENTS CONTAIN:

## Document: 04_CAMERA_AND_TOF_DRIVERS.md
- IMX500 RGB camera driver
- Arducam ToF depth camera driver
- Camera info publishers
- ROS 2 sensor interfaces

## Document: 05_STATIC_TF_PUBLISHER.md
- Static transform publisher for entire TF tree
- base_link → camera_link, tof_link, imu_link
- URDF-based static transform setup

## Document: 06_EKF_FUSION_NODE.md
- robot_localization EKF integration
- Odometry fusion from VEX + IMU
- Full YAML configuration with tuning guide

## Document: 07_PERCEPTION_NODES.md
- IMX500 classifier node
- YOLO detector node
- Landmark 3D localizer
- Auto-exposure controller

## Document: 08_LOCALIZATION_NODES.md
- Landmark map server
- Mission health monitor
- Landmark verification node

## Document: 09_NAVIGATION_NODES.md
- Waypoint recorder/player
- Terrain costmap layer plugin
- Mission logger node
- Streaming node (MJPEG)

## Document: 10_LAUNCH_FILES.md
- Phase 0-11 launch files (ROS 2 Python)
- Full mission launch
- Testing and debugging launches

## Document: 11_CONFIGURATION_REFERENCE.md
- Complete YAML config files (centralized)
- RTAB-Map tuning guide
- Nav2 parameter reference
- Calibration procedures

## Document: 12_BUILD_PROCEDURES.md
- Step-by-step workspace setup
- Dependency installation
- Colcon build instructions
- Troubleshooting guide

---

# IMMEDIATE NEXT STEPS

1. You now have:
   - ✅ IMPLEMENTATION_ROADMAP.md (overall architecture)
   - ✅ 01_ROVER_MSGS.md (message definitions)
   - ✅ 02_VEX_DRIVER.md (wheel odometry driver)
   - ✅ 03_IMU_DRIVER.md (orientation driver)

2. Request the next document (04_CAMERA_AND_TOF_DRIVERS.md)
   To get: IMX500 + ToF drivers + camera publishers

3. Continue sequentially through documents

---

# IMPLEMENTATION STRATEGY

## Recommended Build Order:

### Week 1: Hardware Bringup
1. Create workspace (Phase 0)
2. Build rover_msgs
3. Build vex_driver_node → verify /vex/odom_raw published
4. Build imu_driver_node → verify /imu/data published
5. Build camera drivers → verify /camera/image_raw, /tof/points
6. Build static TF → verify TF tree correct

### Week 2: Sensor Fusion
1. Build EKF fusion → verify /odom published at 50 Hz
2. Tune EKF parameters with actual sensor data
3. Run Phase 4 with camera only
4. Add ToF depth to RTAB-Map

### Week 3: Perception & Navigation
1. Add classifiers + YOLO
2. Build mission health monitor
3. Build Nav2 with static costmap
4. Integrate waypoint recording

### Week 4: Integration & Testing
1. Full system integration (Phase 11)
2. Mission logging
3. Replay and validation
4. Calibration refinement

---

# KEY FILE LOCATIONS

After all documents are generated, your workspace will look like:

```
~/ros2_moon_rover/
├── src/
│   ├── rover_msgs/                    (custom message definitions)
│   ├── vex_driver_node/               (wheel encoders)
│   ├── imu_driver_node/               (accelerometer + gyro)
│   ├── imx500_camera_node/            (RGB camera driver)
│   ├── tof_camera_node/               (depth camera driver)
│   ├── static_tf_publisher/           (TF transforms)
│   ├── ekf_fusion_node/               (odometry fusion - robot_localization)
│   ├── auto_exposure_controller_node/ (camera parameter control)
│   ├── imx500_classifier_node/        (on-device AI classifier)
│   ├── yolo_detector_node/            (landmark detection)
│   ├── landmark_localizer_node/       (3D position estimation)
│   ├── landmark_map_server/           (persistent landmark storage)
│   ├── mission_health_monitor/        (SLAM health tracking)
│   ├── waypoint_recorder_node/        (trajectory recording)
│   ├── waypoint_player_node/          (trajectory replay)
│   ├── terrain_layer_plugin/          (Nav2 costmap plugin)
│   ├── nav2_integration/              (Nav2 configuration)
│   ├── streaming_node/                (MJPEG video server)
│   ├── mission_logger_node/           (rosbag + events)
│   └── rover_launch/                  (11 phase launches + configs)
│
├── configs/                           (centralized YAML configs)
│   ├── ekf_params.yaml
│   ├── rtabmap_params.yaml
│   ├── nav2_params.yaml
│   ├── camera_params.yaml
│   └── terrain_layer_params.yaml
│
├── colcon_workspace_setup.sh
├── BUILD_INSTRUCTIONS.md
└── CALIBRATION_GUIDE.md
```

---

# CRITICAL IMPLEMENTATION NOTES

## 1. Dependencies You MUST Install
```bash
# Core ROS 2
sudo apt-get install ros-humble-ros-core \
  ros-humble-rclcpp \
  ros-humble-rclpy \
  ros-humble-geometry2 \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-tf2-ros \
  ros-humble-cv-bridge \
  ros-humble-image-transport

# Python packages for Pi
pip3 install opencv-python numpy scipy

# Build tools
sudo apt-get install build-essential cmake git
```

## 2. Source Packages (Clone to src/)
```bash
cd ~/ros2_moon_rover/src

# SLAM (MUST build from source)
git clone --branch humble https://github.com/ros-perception/rtabmap_ros.git
git clone --branch humble https://github.com/introlab/rtabmap.git

# Navigation (MUST build from source)
git clone --branch humble https://github.com/ros-planning/navigation2.git
git clone --branch humble https://github.com/ros-planning/navigation2_msgs.git

# Utilities
git clone --branch humble https://github.com/ros-perception/image_common.git
git clone --branch humble https://github.com/ros-perception/diagnostics.git

# EKF (robot_localization) - MUST build from source
git clone --branch humble https://github.com/cra-ros-pkg/robot_localization.git
```

## 3. Build Order (CRITICAL)
```bash
# Step 1: Build messages first
colcon build --packages-select rover_msgs --symlink-install

# Step 2: Build hardware drivers (independent)
colcon build --packages-select \
  vex_driver_node \
  imu_driver_node \
  imx500_camera_node \
  tof_camera_node \
  static_tf_publisher \
  --symlink-install

# Step 3: Build dependencies (robot_localization for EKF)
colcon build --packages-select robot_localization --symlink-install

# Step 4: Build EKF fusion
colcon build --packages-select ekf_fusion_node --symlink-install

# Step 5: Build perception nodes (can be parallel)
colcon build --packages-select \
  imx500_classifier_node \
  yolo_detector_node \
  landmark_localizer_node \
  auto_exposure_controller_node \
  --symlink-install

# Step 6: Build SLAM (depends on robot_localization built)
colcon build --packages-select rtabmap_ros --symlink-install

# Step 7: Build localization/navigation
colcon build --packages-select \
  landmark_map_server \
  mission_health_monitor \
  landmark_verification_node \
  --symlink-install

# Step 8: Build navigation2
colcon build --packages-select navigation2 --symlink-install

# Step 9: Build terrain layer plugin
colcon build --packages-select terrain_layer_plugin --symlink-install

# Step 10: Build monitoring and launch
colcon build --packages-select \
  waypoint_recorder_node \
  waypoint_player_node \
  mission_logger_node \
  streaming_node \
  rover_launch \
  --symlink-install

# Full build
colcon build --symlink-install
```

## 4. Testing Each Phase
```bash
source ~/ros2_moon_rover/install/setup.bash

# Phase 1: Sensors
ros2 launch rover_launch phase_1_sensors.launch.py
# Verify: ros2 topic list shows all sensor topics

# Phase 2: TF
ros2 launch rover_launch phase_2_tf.launch.py
# Verify: ros2 run tf2_tools view_frames.py && evince frames.pdf

# Phase 3: EKF
ros2 launch rover_launch phase_3_ekf.launch.py
# Verify: ros2 topic echo /odom shows smooth odometry

# Phase 4-11: Follow phased bring-up
```

---

# EMERGENCY BUILD REFERENCE

If you get stuck on dependencies:

```bash
# Clean slate
rm -rf ~/ros2_moon_rover/build ~/ros2_moon_rover/install

# Ensure core packages
source /opt/ros/humble/setup.bash

# Install missing dependencies
rosdep install --from-paths ~/ros2_moon_rover/src --ignore-src -r -y

# Try single package
colcon build --packages-select rover_msgs --symlink-install

# Debug build
colcon build --packages-select <package> --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

---

# NEXT DOCUMENT REQUEST

Ready for **04_CAMERA_AND_TOF_DRIVERS.md**?

This will contain:
- [ ] IMX500 camera driver (libcamera integration)
- [ ] ToF camera driver (depth depth point clouds)
- [ ] CameraInfo publisher
- [ ] Image encoding + compression
- [ ] Build instructions specific to Raspberry Pi

Say "next" or "Continue with 04" to get that file.
