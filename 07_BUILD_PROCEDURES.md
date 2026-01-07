# ROS 2 Moon Rover - Complete Build & Test Procedures

## ⚠️ CRITICAL PREREQUISITES

### System Requirements:
- Raspberry Pi 5 (8GB+ RAM recommended)
- Raspberry Pi OS Bookworm (latest)
- ROS 2 Humble already installed

### Check ROS 2 installation:
```bash
source /opt/ros/humble/setup.bash
ros2 --version
# Should output: ROS 2 humble
```

---

# STEP 1: CREATE WORKSPACE

```bash
# Create workspace directory
mkdir -p ~/ros2_moon_rover/src
cd ~/ros2_moon_rover

# Initialize workspace structure
colcon build --help  # Verify colcon is installed
```

---

# STEP 2: CLONE SOURCE PACKAGES (CRITICAL!)

These packages MUST be built from source:

```bash
cd ~/ros2_moon_rover/src

# SLAM - rtabmap_ros (this is THE SLAM engine)
git clone --branch humble https://github.com/ros-perception/rtabmap_ros.git

# SLAM - rtabmap core library
git clone --branch humble https://github.com/introlab/rtabmap.git

# Navigation - nav2 and dependencies
git clone --branch humble https://github.com/ros-planning/navigation2.git
git clone --branch humble https://github.com/ros-planning/navigation2_msgs.git

# Odometry fusion - robot_localization (EKF)
git clone --branch humble https://github.com/cra-ros-pkg/robot_localization.git

# Utilities
git clone --branch humble https://github.com/ros-perception/image_common.git
git clone --branch humble https://github.com/ros-perception/diagnostics.git

# Verify clones
ls -la
# Should show: rtabmap, rtabmap_ros, navigation2, navigation2_msgs, robot_localization, image_common, diagnostics
```

---

# STEP 3: COPY YOUR CUSTOM PACKAGES

From the generated documents above, copy all packages into src/:

```bash
cd ~/ros2_moon_rover/src

# Create package directories (copy code from documents into these)
mkdir -p rover_msgs
mkdir -p vex_driver_node
mkdir -p imu_driver_node
mkdir -p imx500_camera_node
mkdir -p tof_camera_node
mkdir -p static_tf_publisher
mkdir -p ekf_fusion_node
mkdir -p imx500_classifier_node
mkdir -p yolo_detector_node
mkdir -p landmark_localizer_node
mkdir -p landmark_map_server
mkdir -p mission_health_monitor
mkdir -p waypoint_recorder_node
mkdir -p waypoint_player_node
mkdir -p terrain_layer_plugin
mkdir -p auto_exposure_controller_node
mkdir -p landmark_verification_node
mkdir -p streaming_node
mkdir -p mission_logger_node
mkdir -p rover_launch

# Copy generated files into each directory
# (CMakeLists.txt, package.xml, src/, config/, launch/, include/)
```

---

# STEP 4: INSTALL SYSTEM DEPENDENCIES

```bash
# Update system
sudo apt-get update
sudo apt-get upgrade -y

# ROS 2 core packages (already installed, but verify)
sudo apt-get install -y \
  ros-humble-ros-core \
  ros-humble-rclcpp \
  ros-humble-rclpy \
  ros-humble-geometry2 \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-message-generation \
  ros-humble-rosidl-cmake

# Pi-specific libraries
sudo apt-get install -y \
  libopencv-dev \
  python3-opencv \
  python3-numpy \
  python3-scipy \
  libcamera-dev \
  python3-libcamera \
  libcamera-tools \
  i2c-tools \
  libi2c-dev

# Build tools
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions

# Optional: YOLO support
pip3 install --upgrade pip
pip3 install ultralytics opencv-python numpy torch torchvision
```

---

# STEP 5: RESOLVE DEPENDENCIES

```bash
cd ~/ros2_moon_rover

# Use rosdep to install any missing dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

# STEP 6: BUILD WORKSPACE (CORRECT ORDER!)

**CRITICAL: Build in this exact order!**

```bash
cd ~/ros2_moon_rover

# Step 6a: Build custom message package FIRST
echo "=== Building rover_msgs ==="
colcon build --packages-select rover_msgs --symlink-install
if [ $? -ne 0 ]; then echo "FAILED!"; exit 1; fi

# Step 6b: Build dependencies (rtabmap, navigation2, robot_localization)
echo "=== Building source packages ==="
colcon build --packages-select \
  rtabmap \
  rtabmap_ros \
  robot_localization \
  navigation2 \
  navigation2_msgs \
  --symlink-install
if [ $? -ne 0 ]; then echo "FAILED!"; exit 1; fi

# Step 6c: Build hardware drivers
echo "=== Building hardware drivers ==="
colcon build --packages-select \
  vex_driver_node \
  imu_driver_node \
  imx500_camera_node \
  tof_camera_node \
  static_tf_publisher \
  --symlink-install
if [ $? -ne 0 ]; then echo "FAILED!"; exit 1; fi

# Step 6d: Build localization/odometry
echo "=== Building localization ==="
colcon build --packages-select \
  ekf_fusion_node \
  landmark_localizer_node \
  landmark_map_server \
  landmark_verification_node \
  --symlink-install
if [ $? -ne 0 ]; then echo "FAILED!"; exit 1; fi

# Step 6e: Build perception/monitoring
echo "=== Building perception ==="
colcon build --packages-select \
  imx500_classifier_node \
  yolo_detector_node \
  mission_health_monitor \
  auto_exposure_controller_node \
  --symlink-install
if [ $? -ne 0 ]; then echo "FAILED!"; exit 1; fi

# Step 6f: Build navigation/recording
echo "=== Building navigation ==="
colcon build --packages-select \
  terrain_layer_plugin \
  waypoint_recorder_node \
  waypoint_player_node \
  streaming_node \
  mission_logger_node \
  --symlink-install
if [ $? -ne 0 ]; then echo "FAILED!"; exit 1; fi

# Step 6g: Build launch files
echo "=== Building launch package ==="
colcon build --packages-select rover_launch --symlink-install
if [ $? -ne 0 ]; then echo "FAILED!"; exit 1; fi

# Step 6h: FULL BUILD (catches any remaining issues)
echo "=== Full build ==="
colcon build --symlink-install
if [ $? -ne 0 ]; then echo "FULL BUILD FAILED"; exit 1; fi

echo "BUILD COMPLETE!"
```

**Troubleshooting build errors:**

```bash
# If a specific package fails:
colcon build --packages-select <package_name> --symlink-install 2>&1 | tail -100

# Clean and retry
rm -rf build install
colcon build --packages-select rover_msgs --symlink-install
```

---

# STEP 7: SOURCE WORKSPACE

```bash
# Add to ~/.bashrc for persistence
echo "source ~/ros2_moon_rover/install/setup.bash" >> ~/.bashrc

# Source for current session
source ~/ros2_moon_rover/install/setup.bash

# Verify
ros2 pkg list | grep rover
# Should list all rover_* packages
```

---

# STEP 8: PHASED TESTING

### **PHASE 1: Sensor Bringup** ✅ FIRST TEST

```bash
# Terminal 1: Launch sensors
source ~/ros2_moon_rover/install/setup.bash
ros2 launch rover_launch phase_1_sensors.launch.py

# Terminal 2: Monitor topics
source ~/ros2_moon_rover/install/setup.bash
ros2 topic list

# Expected output:
# /vex/odom_raw
# /imu/data
# /image_raw
# /camera_info
# /tof/depth/image_raw
# /tof/points

# Test each sensor individually
ros2 topic echo /vex/odom_raw --rate 1
ros2 topic echo /imu/data --rate 1
ros2 topic echo /camera/camera_info --rate 1
```

**Definition of Done:** All 4 sensors publishing data at expected rates ✅

---

### **PHASE 2: TF Tree Verification** ✅ SECOND TEST

```bash
# Terminal 1: Launch TF
ros2 launch rover_launch phase_2_tf.launch.py

# Terminal 2: View frames
source ~/ros2_moon_rover/install/setup.bash
ros2 run tf2_tools view_frames.py
evince frames.pdf

# Expected tree:
# map
#  └─ odom (published by EKF in Phase 3)
#      └─ base_link (published by EKF)
#          ├─ camera_link
#          ├─ tof_link
#          └─ imu_link
```

**Definition of Done:** TF tree matches spec exactly ✅

---

### **PHASE 3: EKF Odometry Fusion** ✅ THIRD TEST

```bash
# Launch phase 1 + 2 + EKF
ros2 launch rover_launch phase_3_ekf.launch.py

# Monitor fused odometry
ros2 topic echo /odom

# Check covariance (should be low and stable)
# x position variance should be < 0.1
# y position variance should be < 0.1
# yaw variance should be < 0.05

# Record test
ros2 bag record -o phase3_test /odom /vex/odom_raw /imu/data
# Move rover 1 meter forward, stop
# Check drift: current odometry should match distance traveled
```

**Definition of Done:** Odometry drift < 0.5 m over 10 m traverse ✅

---

### **PHASE 4: RTAB-Map SLAM** ✅ FOURTH TEST

```bash
# Launch phase 1-3 + RTAB-Map
ros2 launch rover_launch phase_4_rtabmap.launch.py

# Open RViz
rviz2

# In RViz:
# - Add: /rtabmap/mapData (default visualization)
# - Add: /camera/image_raw (check camera feed)
# - Add: /tof/points (check depth)
# - Set Fixed Frame to: map

# Move rover in loop and watch map build
# Should see loop closure triggers when returning to start
```

**Definition of Done:** Map builds and loop closure detected ✅

---

### **PHASE 5-11: Integration Testing**

Continue through phases sequentially:

```bash
# Phase 5: Terrain layer
ros2 launch rover_launch phase_5_terrain_layer.launch.py
# Check /move_base_simple/goal accepts goals, rover plans paths

# Phase 6: Auto-exposure
ros2 launch rover_launch phase_6_auto_exposure.launch.py
# Watch image brightness stabilize in variable lighting

# Phase 7: Perception
ros2 launch rover_launch phase_7_perception.launch.py
# Monitor /landmarks/detections and /landmarks/positions
ros2 topic echo /landmarks/positions

# Phase 8-9: Navigation
ros2 launch rover_launch phase_9_nav2_rtabmap.launch.py
# In RViz, set 2D Nav Goal
# Watch rover navigate to goal

# Phase 10: Waypoint recording
ros2 launch rover_launch phase_10_waypoint_recording.launch.py
# Record a path, save it
ros2 service call /waypoint_recorder/save_waypoints std_srvs/srv/Trigger
# Replay saved path

# Phase 11: Full system
ros2 launch rover_launch phase_11_full_mission.launch.py
# Everything together!
```

---

# STEP 9: CALIBRATION (CRITICAL!)

Before full deployment, calibrate:

### Camera Intrinsics
```bash
# For IMX500:
ros2 run camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camera

# Generate calibration file, update camera_params.yaml
```

### IMU Calibration
```bash
# Collect IMU data while stationary:
ros2 bag record /imu/data &
sleep 30
killall ros2

# Calculate offsets from rosbag
python3 - <<'EOF'
import rosbag2_py
# Read bag, compute mean of accel and gyro
# Update imu_params.yaml with offsets
EOF
```

### Extrinsic Calibration
```bash
# Use checkerboard to verify static transforms
# Project known landmark in camera frame to map
# If projection error > 10cm, adjust static transforms
```

---

# STEP 10: MISSION EXECUTION

```bash
# Source workspace
source ~/ros2_moon_rover/install/setup.bash

# Launch full system
ros2 launch rover_launch phase_11_full_mission.launch.py &

# Wait for SLAM to initialize (~30 seconds)
sleep 30

# Open mission control (RViz)
rviz2 &

# Set waypoint via RViz 2D Nav Goal
# Monitor /mission_health for status
ros2 topic echo /mission_health

# Rover will autonomously navigate!
```

---

# EMERGENCY FIXES

**Build fails with "rtabmap not found":**
```bash
# Ensure rtabmap is built first
colcon build --packages-select rtabmap --symlink-install
colcon build --packages-select rtabmap_ros --symlink-install
# Then retry full build
```

**Sensors not publishing:**
```bash
# Check hardware connections
i2cdetect -y 1  # Should show IMU at 0x68
ls -la /dev/ttyUSB*  # VEX should appear here

# Check permissions
sudo usermod -a -G i2c $USER
sudo usermod -a -G dialout $USER
# Log out and back in
```

**TF errors:**
```bash
# Verify all nodes launched
ros2 node list

# If missing nodes:
ros2 launch rover_launch phase_2_tf.launch.py &
# Then check frames.pdf again
```

**Low FPS/performance:**
```bash
# Reduce resolution
# camera: 640x480 → 320x240
# tof: 320x240 → 160x120

# Reduce publishing rates
# camera: 30 fps → 15 fps
# imu: 100 hz → 50 hz
```

---

# PERFORMANCE TARGETS

| Metric | Target | Acceptable |
|--------|--------|-----------|
| Sensor data latency | <50 ms | <100 ms |
| Odometry drift | <0.5 m/10 m | <1 m/10 m |
| SLAM loop closure | Every 5-10 m | Every 15 m |
| Navigation success | >90% | >70% |
| Full system CPU | <70% | <85% |
| Temperature | <60°C | <70°C |

---

# SUCCESS CHECKLIST

- [ ] All packages build without errors
- [ ] Phase 1: All 4 sensors publishing
- [ ] Phase 2: TF tree correct
- [ ] Phase 3: Odometry fused and stable
- [ ] Phase 4: RTAB-Map builds map
- [ ] Phase 5: Terrain costmap populated
- [ ] Phase 6: Auto-exposure adjusting brightness
- [ ] Phase 7: Landmarks detected and localized
- [ ] Phase 8-9: Navigation2 planning paths
- [ ] Phase 10: Waypoints recorded and replayed
- [ ] Phase 11: Full autonomous mission executes
- [ ] Rover completes 50 m mission without intervention

---

# NEXT STEPS

1. **Build workspace** (Steps 1-7 above)
2. **Test Phase 1** sensors, fix hardware connections if needed
3. **Incrementally test Phases 2-11**
4. **Calibrate** cameras and IMU with actual data
5. **Tune parameters** based on real rover performance
6. **Run mission** autonomously!

**Estimated time to full integration: 2-3 weeks with testing**

Good luck! 🚀
