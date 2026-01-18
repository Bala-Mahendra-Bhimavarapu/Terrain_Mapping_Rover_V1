# 🚀 Lunar Rover ROS 2 System - Complete Build Guide

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Phase 0: Workspace Setup](#phase-0-workspace-setup)
3. [Phase 1: Sensor Bring-Up](#phase-1-sensor-bring-up)
4. [Phase 2: TF Setup](#phase-2-tf-setup)
5. [Phase 3: EKF Fusion](#phase-3-ekf-fusion)
6. [Phase 4: RTAB-Map SLAM](#phase-4-rtab-map-slam)
7. [Phase 5: ToF + Terrain Layer](#phase-5-tof--terrain-layer)
8. [Phase 6: Auto-Exposure](#phase-6-auto-exposure)
9. [Phase 7: Classification + Health](#phase-7-classification--health)
10. [Phase 8: Nav2 Static Map](#phase-8-nav2-static-map)
11. [Phase 9: Nav2 + RTAB-Map](#phase-9-nav2--rtab-map)
12. [Phase 10: Waypoint Recording](#phase-10-waypoint-recording)
13. [Phase 11: Full Mission](#phase-11-full-mission)
14. [Phase 12: Bluetooth Navigation](#phase-12-bluetooth-navigation)
15. [Troubleshooting](#troubleshooting)
16. [Calibration Guide](#calibration-guide)

---

## Prerequisites

### Hardware Requirements
- Raspberry Pi 5 (8GB or 16GB RAM recommended)
- Pi AI Camera (IMX500)
- Arducam ToF Camera (B0410)
- MPU6050 IMU
- VEX V5 Brain + Motors
- USB cable connecting Pi to V5
- Adequate cooling (fan + heatsink)
- 128GB+ SD card (preferably high-speed)

### Software Requirements
- Raspberry Pi OS Bookworm 64-bit
- ROS 2 Humble (already built from source)
- Python 3.10+
- GCC/G++ for C++ compilation

---

## Phase 0: Workspace Setup

### Step 1: Create Workspace

```bash
# Create workspace directory
mkdir -p ~/terrain_mapping_rover_ws/src
cd ~/terrain_mapping_rover_ws

# Create VEX V5 code directory (separate from ROS)
mkdir -p ~/vex_v5_code
```

### Step 2: Clone/Copy All Packages

Copy all the package directories I provided into `~/terrain_mapping_rover_ws/src/`:

```
src/
├── rover_interfaces/
├── rover_description/
├── vex_serial/
├── imu_driver/
├── imx500_camera/
├── arducam_tof/
├── sensor_fusion/
├── rover_teleop/
├── auto_exposure/
├── mission_health/
├── waypoint_manager/
├── landmark_system/
├── rover_streaming/
├── bluetooth_nav/
├── rover_bringup/
```

### Step 3: Clone External Dependencies

```bash
cd ~/terrain_mapping_rover_ws/src

# Robot Localization (EKF)
git clone -b humble https://github.com/cra-ros-pkg/robot_localization. git

# RTAB-Map
git clone -b humble-devel https://github.com/introlab/rtabmap_ros. git
git clone -b humble-devel https://github.com/introlab/rtabmap. git

# Navigation2
git clone -b humble https://github.com/ros-planning/navigation2.git

# Nav2 dependencies
git clone -b humble https://github.com/BehaviorTree/BehaviorTree. CPP.git
git clone -b humble https://github.com/ros/bond_core.git

# Image transport
git clone -b humble https://github.com/ros-perception/image_common.git
git clone -b humble https://github.com/ros-perception/vision_opencv.git

# TF2
git clone -b humble https://github.com/ros2/geometry2.git

# Xacro
git clone -b humble https://github.com/ros/xacro.git
```

### Step 4: Install Python Dependencies

```bash
# System packages
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    python3-serial \
    python3-smbus2 \
    python3-flask \
    i2c-tools \
    libopencv-dev \
    libeigen3-dev \
    libpcl-dev \
    libgtsam-dev

# Python packages
pip3 install --user \
    pyserial \
    smbus2 \
    flask \
    ultralytics \
    scipy \
    transforms3d

# Optional:  Bluetooth (requires root for scanning)
pip3 install --user bluepy
```

### Step 5: Install Arducam ToF SDK

```bash
# Clone Arducam ToF SDK
cd ~
git clone https://github.com/ArduCAM/Arducam_tof_camera.git
cd Arducam_tof_camera

# Install SDK
./Install_dependencies.sh
mkdir build && cd build
cmake ..
make -j4
sudo make install

# Install Python bindings
cd ../python
pip3 install --user .
```

### Step 6: Install Picamera2

```bash
sudo apt install -y python3-picamera2 python3-libcamera
```

### Step 7: Build the Workspace

```bash
cd ~/terrain_mapping_rover_ws

# Source ROS 2
source /opt/ros/humble/setup.bash
# OR if built from source: 
source ~/ros2_humble/install/setup.bash

# Install rosdep dependencies
sudo rosdep init  # Only if not done before
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build (this will take a while!)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# If you run out of memory, build with limited parallelism: 
colcon build --symlink-install --parallel-workers 2
```

### Step 8: Source the Workspace

Add to your `~/.bashrc`:

```bash
echo "source ~/terrain_mapping_rover_ws/install/setup.bash" >> ~/.bashrc
source ~/. bashrc
```

### Step 9: Set Up VEX V5 Code

Copy the VEX V5 code to your development computer:

```bash
# On your development computer (not the Pi)
# Copy the vex_v5_code directory

# Open VS Code with PROS extension
# 1. Open the vex_v5_code folder
# 2. Use PROS extension to build
# 3. Upload to V5 Brain via USB
```

### Step 10: Configure Permissions

```bash
# Serial port access
sudo usermod -a -G dialout $USER

# I2C access
sudo usermod -a -G i2c $USER

# Enable I2C
sudo raspi-config
# Navigate to:  Interface Options -> I2C -> Enable

# Camera access
sudo usermod -a -G video $USER

# Bluetooth (for beacon scanning)
sudo setcap 'cap_net_raw,cap_net_admin+eip' $(which python3)

# Reboot for changes to take effect
sudo reboot
```

---

## Phase 1: Sensor Bring-Up

### Definition of Done
- [ ] VEX Serial node connects and receives odometry
- [ ] IMU publishes data at ~100Hz
- [ ] Camera publishes images at ~30Hz
- [ ] ToF publishes depth at ~15Hz
- [ ] All topics visible with `ros2 topic list`

### Step 1: Test VEX Serial Connection

```bash
# Check USB connection
ls -la /dev/ttyACM*

# Launch VEX serial node only
ros2 run vex_serial vex_serial_node.py

# In another terminal, check topics
ros2 topic list
ros2 topic hz /vex/odom_raw
ros2 topic echo /vex/odom_raw
```

**Expected:** `/vex/odom_raw` publishing at 50Hz

### Step 2: Test IMU

```bash
# Check I2C connection
i2cdetect -y 1
# Should show device at 0x68

# Launch IMU node
ros2 run imu_driver mpu6050_node.py

# Check output
ros2 topic hz /imu/data
ros2 topic echo /imu/data
```

**Expected:** `/imu/data` publishing at ~100Hz

### Step 3: Test Camera

```bash
# Launch camera node
ros2 run imx500_camera imx500_camera_node.py

# Check output
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info
```

**Expected:** `/camera/image_raw` at ~30Hz

### Step 4: Test ToF

```bash
# Launch ToF node
ros2 run arducam_tof arducam_tof_node.py

# Check output
ros2 topic hz /tof/depth/image_raw
ros2 topic hz /tof/points
```

**Expected:** `/tof/depth/image_raw` at ~15Hz

### Step 5: Launch All Sensors Together

```bash
# Using mock mode for testing without hardware
ros2 launch rover_bringup phase1_sensors.launch.py use_mock: =true

# With real hardware
ros2 launch rover_bringup phase1_sensors. launch.py use_mock:=false
```

### Troubleshooting Phase 1

**VEX not connecting:**
```bash
# Check USB connection
dmesg | tail -20
# Try different USB port
# Make sure V5 Brain is running the uploaded program
```

**IMU not found:**
```bash
# Check I2C
i2cdetect -y 1
# If 0x68 not shown, check wiring
# If shown as 0x69, change address in config
```

**Camera not working:**
```bash
# Check camera connection
vcgencmd get_camera
# Should show:  supported=1 detected=1
```

---

## Phase 2: TF Setup

### Definition of Done
- [ ] TF tree visible in RViz
- [ ] All frames connected:  base_link → camera_link, tof_link, imu_link
- [ ] `ros2 run tf2_tools view_frames` shows correct tree

### Step 1: Launch Description

```bash
ros2 launch rover_bringup phase2_tf.launch.py
```

### Step 2: Verify TF Tree

```bash
# View TF tree
ros2 run tf2_tools view_frames
# Creates frames. pdf

# Check specific transforms
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo base_link tof_link
ros2 run tf2_ros tf2_echo base_link imu_link
```

### Step 3: Visualize in RViz

```bash
# Launch RViz
rviz2 -d ~/terrain_mapping_rover_ws/src/rover_bringup/rviz/rover_default.rviz

# Set Fixed Frame to "base_link"
# Enable TF display
# Verify all sensor frames are correctly positioned
```

---

## Phase 3: EKF Fusion

### Definition of Done
- [ ] `/odom` topic publishing fused odometry
- [ ] TF `odom → base_link` is published
- [ ] Odometry is smooth when driving
- [ ] Covariance values are reasonable

### Step 1: Launch EKF

```bash
# Launch sensors + TF + EKF
ros2 launch rover_bringup phase3_ekf.launch.py use_mock:=false
```

### Step 2: Verify Fusion

```bash
# Check fused odometry
ros2 topic echo /odom

# Check TF
ros2 run tf2_ros tf2_echo odom base_link

# Check update rate
ros2 topic hz /odom
```

**Expected:** `/odom` at ~50Hz with smooth pose updates

### Step 3: Test with Teleop

```bash
# In terminal 1: Launch EKF
ros2 launch rover_bringup phase3_ekf. launch.py

# In terminal 2: Launch teleop
ros2 run rover_teleop keyboard_teleop_node.py

# Drive the rover and watch odometry in RViz
```

### Tuning EKF

Edit `src/sensor_fusion/config/ekf_params.yaml`:

```yaml
# If odometry drifts, increase wheel odom weight: 
odom0_config:  [true, true, false, ...]  # More position trust

# If too noisy, decrease IMU weight:
imu0_config: [false, false, false, ...]  # Less acceleration trust

# Process noise:  higher = trust measurements more
process_noise_covariance: [...]
```

---

## Phase 4: RTAB-Map SLAM

### Definition of Done
- [ ] RTAB-Map node running
- [ ] `/map` topic publishing occupancy grid
- [ ] TF `map → odom` published by RTAB-Map
- [ ] Map builds as rover moves

### Step 1: Configure RTAB-Map

Verify settings in `src/rover_bringup/config/rtabmap_params.yaml`

### Step 2: Launch RTAB-Map

```bash
# First, launch sensors + EKF
ros2 launch rover_bringup phase3_ekf.launch.py

# Then, launch RTAB-Map (in another terminal)
ros2 launch rtabmap_ros rtabmap. launch.py \
    frame_id:=base_link \
    subscribe_depth:=true \
    approx_sync:=true \
    rgb_topic:=/camera/image_raw \
    depth_topic: =/tof/depth/image_raw \
    camera_info_topic:=/camera/camera_info \
    odom_topic:=/odom
```

### Step 3: Verify Mapping

```bash
# Check map topic
ros2 topic echo /map --once

# View in RViz
# Add Map display, set topic to /map
```

### Troubleshooting RTAB-Map

**No features detected:**
- Increase `Kp/MaxFeatures` in config
- Check camera exposure
- Ensure adequate lighting/texture

**Map drifting:**
- Enable loop closure detection
- Add more loop closure parameters
- Check odometry quality

---

## Phase 5: ToF + Terrain Layer

### Definition of Done
- [ ] ToF point cloud visible in RViz
- [ ] Obstacles appear in costmap
- [ ] Nav2 costmap updates with obstacles

### Step 1: Verify ToF Point Cloud

```bash
# Launch sensors
ros2 launch rover_bringup phase3_ekf.launch.py

# Check point cloud in RViz
# Add PointCloud2 display, topic:  /tof/points
```

### Step 2: Configure Costmap

The terrain layer is configured in `nav2_params.yaml` under `obstacle_layer`.

---

## Phase 6: Auto-Exposure

### Definition of Done
- [ ] Auto-exposure node adjusts camera settings
- [ ] `/camera/exposure_state` shows adjustments
- [ ] Images maintain consistent brightness

### Step 1: Launch Auto-Exposure

```bash
# Launch with camera
ros2 launch imx500_camera camera. launch.py

# Launch auto-exposure
ros2 launch auto_exposure auto_exposure. launch.py
```

### Step 2: Verify

```bash
ros2 topic echo /auto_exposure/state
ros2 topic echo /camera/exposure_state
```

---

## Phase 7: Classification + Health

### Definition of Done
- [ ] Mission health node publishing status
- [ ] Health status reflects subsystem states
- [ ] Events published when status changes

### Step 1: Launch Health Monitor

```bash
ros2 launch mission_health mission_health.launch.py
```

### Step 2: Verify

```bash
ros2 topic echo /mission_health
ros2 topic echo /mission_events
```

---

## Phase 8: Nav2 Static Map

### Definition of Done
- [ ] Nav2 stack running
- [ ] Can send goal poses
- [ ] Robot navigates to goals

### Step 1: Save a Map

```bash
# After RTAB-Map has built a map
ros2 run nav2_map_server map_saver_cli -f ~/maps/test_map
```

### Step 2: Launch Nav2 with Static Map

```bash
ros2 launch nav2_bringup bringup_launch.py \
    map: =~/maps/test_map. yaml \
    params_file:=~/terrain_mapping_rover_ws/src/rover_bringup/config/nav2_params.yaml
```

### Step 3: Send Navigation Goal

In RViz:
1. Click "2D Goal Pose" tool
2. Click and drag on map to set goal
3. Robot should navigate to goal

---

## Phase 9: Nav2 + RTAB-Map

### Definition of Done
- [ ] Nav2 uses live RTAB-Map map
- [ ] Navigation works without pre-built map
- [ ] Full autonomous exploration possible

---

## Phase 10: Waypoint Recording

### Definition of Done
- [ ] Can start/stop waypoint recording
- [ ] Waypoints saved to file
- [ ] Can replay waypoints

### Step 1: Record Waypoints

```bash
# Launch waypoint manager
ros2 launch waypoint_manager waypoint. launch.py

# Start recording (use service)
ros2 service call /waypoint_recorder/start std_srvs/srv/SetBool "{data: true}"

# Drive the rover around... 

# Stop recording
ros2 service call /waypoint_recorder/stop std_srvs/srv/Trigger

# Save waypoints
ros2 service call /waypoint_recorder/save std_srvs/srv/Trigger
```

### Step 2: Replay Waypoints

```bash
# Load waypoints
ros2 service call /waypoint_follower/load std_srvs/srv/Trigger

# Start following
ros2 service call /waypoint_follower/start std_srvs/srv/Trigger
```

---

## Phase 11: Full Mission

### Definition of Done
- [ ] All systems running together
- [ ] Autonomous navigation working
- [ ] Waypoints recorded and replayed
- [ ] Data logged for analysis

### Launch Full System

```bash
ros2 launch rover_bringup full_system.launch.py
```

---

## Phase 12: Bluetooth Navigation

### Definition of Done
- [ ] Beacon scanner detects beacons
- [ ] Navigator moves toward beacon
- [ ] Rover arrives at beacon location

### Step 1: Configure Beacon

Set target beacon UUID in config file. 

### Step 2: Launch Bluetooth Navigation

```bash
ros2 launch bluetooth_nav bluetooth_nav.launch. py enable_navigation:=true
```

---

## Troubleshooting

### Common Issues

**Build fails with memory error:**
```bash
# Reduce parallel jobs
colcon build --parallel-workers 1
# Or add swap
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

**TF timeout errors:**
```bash
# Increase TF buffer time in nodes
# Check that all transforms are being published
ros2 run tf2_tools view_frames
```

**Robot not moving:**
```bash
# Check VEX connection
ros2 topic echo /vex/status
# Check cmd_vel is being published
ros2 topic echo /vex/cmd_vel
```

**SLAM not working:**
```bash
# Check image quality
ros2 run rqt_image_view rqt_image_view
# Check depth alignment
# Verify camera intrinsics
```

---

## Calibration Guide

### Camera Intrinsic Calibration

```bash
# Print checkerboard (9x6 inner corners, 25mm squares)
# Take 20+ images from different angles

# Use camera_calibration package
ros2 run camera_calibration cameracalibrator \
    --size 9x6 \
    --square 0.025 \
    image:=/camera/image_raw \
    camera: =/camera

# Save calibration and update parameters
```

### ToF Calibration

```bash
# Measure known distances
# Compare with ToF readings
# Adjust depth_scale parameter
```

### Extrinsic Calibration

Measure physical sensor positions and update URDF: 
- `src/rover_description/urdf/properties.urdf.xacro`

### IMU Calibration

IMU calibrates on startup when stationary.  Keep rover still for 5 seconds after boot.

---

## Data Sharing Between Rovers

### Save Mission Data

```bash
# Waypoints are in:  ~/. ros/waypoints/
# Landmark maps in: ~/.ros/landmark_maps/
# RTAB-Map database in: ~/.ros/rtabmap. db
```

### Transfer to Another Rover

```bash
# Copy files via network
scp ~/. ros/waypoints/mission_*. yaml rover2: ~/.ros/waypoints/
scp ~/.ros/landmark_maps/*. yaml rover2:~/.ros/landmark_maps/
```

### Load on Second Rover

```bash
# Set waypoint file parameter
ros2 param set /waypoint_follower_node waypoint_file "mission_xyz.yaml"

# Set landmark map parameter
ros2 param set /landmark_verification_node landmark_map_file "map_xyz.yaml"
```

---

## Quick Reference

### Key Launch Commands

```bash
# Manual drive only
ros2 launch rover_bringup manual_drive.launch.py

# Full autonomous system
ros2 launch rover_bringup full_system.launch. py

# With mock sensors (testing)
ros2 launch rover_bringup phase1_sensors. launch.py use_mock:=true
```

### Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| /vex/odom_raw | Odometry | Raw wheel odometry |
| /imu/data | Imu | IMU data |
| /camera/image_raw | Image | RGB camera |
| /tof/depth/image_raw | Image | Depth image |
| /tof/points | PointCloud2 | 3D points |
| /odom | Odometry | Fused odometry |
| /map | OccupancyGrid | SLAM map |
| /mission_health | MissionHealth | System health |
| /vex/cmd_vel | Twist | Velocity commands |

### Key Services

| Service | Type | Description |
|---------|------|-------------|
| /waypoint_recorder/start | SetBool | Start recording |
| /waypoint_recorder/stop | Trigger | Stop recording |
| /waypoint_follower/start | Trigger | Start following |

---

## Support

If you encounter issues: 
1. Check this troubleshooting guide
2. Verify hardware connections
3. Check ROS logs:  `ros2 topic echo /rosout`
4. Review sensor data in RViz

Good luck with your lunar rover!  🚀