# 06_LAUNCH_FILES.md - Complete Phase-By-Phase Launch Guide
# Updated January 6, 2026 - All 12 Phases, Proper Configuration, Verified Consistency

---

## 🎯 PURPOSE

This document provides **all 11 phase launch files** and their launch commands for the sequential rover bring-up.

**Each phase must be tested and verified before proceeding to the next phase.**

---

## 📋 FOLDER STRUCTURE FOR LAUNCH FILES

All launch files should be located in:
```
~/ros2_moon_rover/src/rover_launch/launch/
```

Configuration files should be located in:
```
~/ros2_moon_rover/src/rover_launch/config/
```

**Verify structure:**
```bash
ls -la ~/ros2_moon_rover/src/rover_launch/
# Should show: launch/, config/, package.xml, setup.py (or CMakeLists.txt)

ls ~/ros2_moon_rover/src/rover_launch/launch/ | sort
# Should show 12 files: phase_0_*.py through phase_11_*.py

ls ~/ros2_moon_rover/src/rover_launch/config/ | sort
# Should show 9+ YAML files
```

---

## 🚀 QUICK LAUNCH COMMAND REFERENCE

```bash
# Always do this first:
source /opt/ros/humble/setup.bash
source ~/ros2_moon_rover/install/setup.bash

# Then choose your phase:
ros2 launch rover_launch phase_0_hardware_check.launch.py
ros2 launch rover_launch phase_1_sensors.launch.py
ros2 launch rover_launch phase_2_tf.launch.py
ros2 launch rover_launch phase_3_ekf.launch.py
ros2 launch rover_launch phase_4_rtabmap.launch.py
ros2 launch rover_launch phase_5_terrain_layer.launch.py
ros2 launch rover_launch phase_6_auto_exposure.launch.py
ros2 launch rover_launch phase_7_perception.launch.py
ros2 launch rover_launch phase_8_nav2_static.launch.py
ros2 launch rover_launch phase_9_nav2_rtabmap.launch.py
ros2 launch rover_launch phase_10_waypoint_recording.launch.py
ros2 launch rover_launch phase_11_full_mission.launch.py
```

---

## 📖 PHASE DOCUMENTATION & LAUNCH COMMANDS

### **PHASE 0: Hardware Check (Prerequisite)**

**File:** `phase_0_hardware_check.launch.py`

**Purpose:** Verify all hardware is connected and accessible before bringing up software.

**Hardware Checked:**
- VEX V5 brain (USB serial)
- IMU (I2C address 0x68)
- IMX500 camera (CSI)
- Arducam ToF camera (CSI)

**Launch Command:**
```bash
ros2 launch rover_launch phase_0_hardware_check.launch.py
```

**Expected Output:**
```
[INFO] Checking VEX V5 brain on /dev/ttyUSB0...
[INFO] VEX brain found ✓
[INFO] Checking IMU on I2C address 0x68...
[INFO] IMU found ✓
[INFO] Checking IMX500 camera...
[INFO] IMX500 camera found ✓
[INFO] Checking Arducam ToF camera...
[INFO] Arducam found ✓
[INFO] All hardware verified!
```

**If Hardware Check Fails:**
1. VEX not found? → Check USB cable, try `ls /dev/ttyUSB*`
2. IMU not found? → Check I2C connection, try `i2cdetect -y 1`
3. Cameras not found? → Check CSI connectors are fully inserted

**Definition of Done:**
- [ ] All 4 hardware components verified
- [ ] No error messages
- [ ] Ready to proceed to Phase 1

---

### **PHASE 1: Sensor Bring-Up (Critical)**

**File:** `phase_1_sensors.launch.py`

**Purpose:** Brings up all 4 hardware drivers and verifies they publish data.

**Nodes Launched:**
1. `vex_driver_node` → publishes `/vex/odom_raw` @ 50 Hz
2. `imu_driver_node` → publishes `/imu/data` @ 100 Hz
3. `imx500_camera_node` → publishes `/camera/image_raw` @ 10-30 fps
4. `tof_camera_node` → publishes `/tof/depth/image_raw` @ 10-20 fps

**Launch Command:**
```bash
ros2 launch rover_launch phase_1_sensors.launch.py
```

**Verify in Another Terminal:**
```bash
# Source workspace first
source /opt/ros/humble/setup.bash
source ~/ros2_moon_rover/install/setup.bash

# Check that all 4 topics are publishing
ros2 topic list | grep -E "vex|imu|camera|tof"
# Should show:
# /camera/camera_info
# /camera/image_raw
# /imu/data
# /tof/depth/image_raw
# /vex/odom_raw

# Check data rates
ros2 topic hz /vex/odom_raw       # Should be ~50 Hz
ros2 topic hz /imu/data           # Should be ~100 Hz
ros2 topic hz /camera/image_raw   # Should be ~10-30 Hz
ros2 topic hz /tof/depth/image_raw # Should be ~10-20 Hz

# View sensor data
ros2 topic echo /imu/data  # Should show acceleration ~9.81 m/s^2 on z-axis
ros2 topic echo /vex/odom_raw  # Should show position changing when rover moves
```

**Using RQT to Visualize:**
```bash
# Terminal 1: Keep Phase 1 running
# Terminal 2: Launch RQT
source ~/ros2_moon_rover/install/setup.bash
rqt &

# In RQT:
# 1. Plugins → Visualization → Image View (subscribe to /camera/image_raw)
# 2. Plugins → Visualization → Plot (plot /imu/data/linear_acceleration/z)
# 3. Plugins → Visualization → Plot (plot /vex/odom_raw/pose/pose/position/x)
```

**Configuration Files Used:**
- `config/vex_params.yaml` — VEX driver parameters
- `config/imu_calibration.yaml` — IMU offsets
- `config/camera_params.yaml` — Camera resolution and frame rate
- `config/tof_params.yaml` — ToF depth range

**Definition of Done:**
- [ ] All 4 topics publishing (verified with `ros2 topic list`)
- [ ] All frequencies correct (verified with `ros2 topic hz`)
- [ ] IMU shows ~9.81 m/s^2 gravity
- [ ] No error messages
- [ ] **Continue to Phase 2 only after verification**

---

### **PHASE 2: TF Tree Setup**

**File:** `phase_2_tf.launch.py`

**Purpose:** Publishes static transforms (base_link → sensors) and verifies TF tree is complete.

**Static Transforms Published:**
```
base_link → camera_link     (+20cm forward, +10cm up)
base_link → tof_link        (+20cm forward, +5cm right, +10cm up)
base_link → imu_link        (center, +5cm up)
```

**Launch Command:**
```bash
ros2 launch rover_launch phase_2_tf.launch.py
```

**Verify in Another Terminal:**
```bash
# Generate TF tree diagram
ros2 run tf2_tools view_frames.py
evince frames.pdf  # View the PDF (or use any PDF viewer)

# Should show tree:
# base_link
#  ├─ camera_link (static)
#  ├─ tof_link (static)
#  └─ imu_link (static)

# Check individual transforms
ros2 run tf2_ros tf2_echo base_link camera_link
# Should show: Translation: [0.200, 0.000, 0.100], Rotation: [0, 0, 0, 1]

ros2 run tf2_ros tf2_echo base_link tof_link
# Should show: Translation: [0.200, 0.050, 0.100], Rotation: [0, 0, 0, 1]

ros2 run tf2_ros tf2_echo base_link imu_link
# Should show: Translation: [0.000, 0.000, 0.050], Rotation: [0, 0, 0, 1]
```

**Configuration Files Used:**
- `config/static_transforms.yaml` — Transform definitions

**Definition of Done:**
- [ ] TF tree complete (3 sensor frames visible)
- [ ] Transform values correct (verify positions match above)
- [ ] No broadcasting errors
- [ ] Ready for Phase 3

---

### **PHASE 3: EKF Odometry Fusion**

**File:** `phase_3_ekf.launch.py`

**Purpose:** Fuses wheel encoders + IMU using Extended Kalman Filter to produce accurate odometry.

**Nodes Launched:**
1. `static_tf_publisher` (from Phase 2)
2. All sensors (from Phase 1)
3. `ekf_fusion_node` → publishes `/odom` @ 50 Hz + TF `odom → base_link`

**New Output:**
- `/odom` (nav_msgs/Odometry) — Fused odometry with covariance
- TF: `odom → base_link`

**Launch Command:**
```bash
ros2 launch rover_launch phase_3_ekf.launch.py
```

**Verify in Another Terminal:**
```bash
# Check EKF is publishing
ros2 topic echo /odom
# Should show position, velocity, and covariance updating

# Check TF broadcasts
ros2 run tf2_ros tf2_echo odom base_link
# Should show position and orientation updating

# Check filter diagnostics
ros2 topic echo /diagnostics
# Should show ekf_filter_node status: OK or WARN
```

**Test the Fusion (Moon Rover Specific):**
```bash
# 1. Place rover on known surface
# 2. Drive rover in small square (4x 1 meter sides, 90° turns)
# 3. Drive back to starting point
# 4. Check final position:

# Record initial position
echo "Initial position:" && ros2 topic echo /odom/pose/pose/position -1 | head -5

# [Drive rover in square...]

# Record final position
echo "Final position:" && ros2 topic echo /odom/pose/pose/position -1 | head -5

# Expected: Final position within ±0.5m of initial (< 5% error)
```

**Configuration Files Used:**
- `config/ekf_params.yaml` — **200+ lines of EKF tuning** (moon-specific!)
  - Process noise (motion model uncertainty)
  - Measurement noise (sensor uncertainty)
  - Initial covariance (starting uncertainty)
  - Wheel radius and separation
  - Gravity on moon vs Earth

**Important Parameters to Tune for Moon:**
```yaml
# From config/ekf_params.yaml:
two_d_mode: true  # 2D localization only
frequency: 50     # Update rate (Hz)

# Process noise (lower = trust motion model more)
process_noise_std: [0.03, 0.05, 0.02]  # for moon (lower gravity)

# Measurement noise (lower = trust sensors more)
imu_acc_std: 0.05    # IMU confidence
odom_pos_std: 0.05   # Odometry confidence
odom_vel_std: 0.05   # Velocity confidence

# Moon-specific gravity
gravitational_acceleration: 1.62  # m/s^2 (vs 9.81 on Earth!)
```

**Definition of Done:**
- [ ] `/odom` topic publishing @ 50 Hz
- [ ] TF `odom → base_link` broadcasting
- [ ] Square drive test: final position < 0.5m from start
- [ ] No diagnostics errors
- [ ] Ready for Phase 4

---

### **PHASE 4: RTAB-Map SLAM**

**File:** `phase_4_rtabmap.launch.py`

**Purpose:** Visual SLAM using camera + depth + wheel odometry to build map and estimate pose.

**Nodes Launched:**
1. All previous (sensors, TF, EKF)
2. `rtabmap_node` → publishes `/rtabmap/mapData` + TF `map → odom`

**New Output:**
- `/rtabmap/mapData` — 3D visual map (point cloud + features)
- TF: `map → odom` — Global frame to odometry frame
- `/rtabmap/info` — SLAM diagnostics

**Launch Command:**
```bash
ros2 launch rover_launch phase_4_rtabmap.launch.py
```

**Visualize in RViz:**
```bash
# Terminal 1: Keep Phase 4 running
# Terminal 2: Launch RViz
source ~/ros2_moon_rover/install/setup.bash
rviz2 &

# In RViz:
# 1. Add → PointCloud2 → /rtabmap/cloud_map (select in dropdown)
# 2. Add → Image → /camera/image_raw (to see what rover sees)
# 3. Add → Image → /tof/depth/image_raw (to see depth)
# 4. Set Global Options → Frame: map
```

**Test SLAM (5-10 minute traverse):**
```bash
# 1. Drive rover slowly in controlled path (avoid fast turns)
# 2. Try to complete a loop (return to starting position)
# 3. Observe RTAB-Map loop closure detection
# 4. Check final map quality in RViz

# Monitor SLAM statistics
ros2 topic echo /rtabmap/info
# Should show:
# - feature_count: 50+ (min 5, better if >20)
# - loop_closure_count: increases when rover revisits areas
```

**Configuration Files Used:**
- `config/rtabmap_params.yaml` — **150+ lines SLAM tuning**
  - Feature detection thresholds (LOW for moon's sparse features)
  - Loop closure parameters (AGGRESSIVE for moon's repetitive terrain)
  - Memory management
  - Depth filtering

**Moon-Specific Parameters:**
```yaml
# From config/rtabmap_params.yaml:
# Moon terrain has sparse visual features, needs aggressive tuning

Mem/BadSignaturesIgnored: false  # Don't ignore bad features
Mem/ImagePreFilter: true          # Pre-filter images
Kp/WordsPerImage: 200             # Extract 200 features (vs default 100)
Kp/DetectorStrategy: 0            # SURF (more features than BRIEF)
Vis/MinInliers: 5                 # Min 5 inliers (vs 15, moon is sparse)
RGBD/ProximityByTime: false       # Don't use time-based loop closure
RGBD/AggressiveLoopClosure: true  # Aggressive loop closure (moon is repetitive!)

# Depth filtering for ToF (moon depth is noisier)
RGBD/MaxLocalRetrieved: 2
RGBD/MaxOdomCacheSize: 200
```

**Definition of Done:**
- [ ] `/rtabmap/mapData` publishing
- [ ] TF `map → odom` broadcasting
- [ ] Point cloud visible in RViz
- [ ] Feature count > 5 (aim for 20+)
- [ ] Loop closure detected on 5-10 minute loop
- [ ] Map quality acceptable (no wild drift)
- [ ] Ready for Phase 5

---

### **PHASE 5: Terrain Layer (Nav2 Costmap)**

**File:** `phase_5_terrain_layer.launch.py`

**Purpose:** Convert ToF depth sensor data to costmap layer for obstacle detection.

**Nodes Launched:**
1. All previous nodes
2. `navigation2` costmap node with terrain layer plugin

**New Output:**
- `/local_costmap/costmap` — Obstacle map
- `/global_costmap/costmap` — Global obstacle map

**Launch Command:**
```bash
ros2 launch rover_launch phase_5_terrain_layer.launch.py
```

**Visualize Costmap in RViz:**
```bash
# Terminal 1: Keep Phase 5 running
# Terminal 2: Launch RViz
source ~/ros2_moon_rover/install/setup.bash
rviz2 &

# In RViz:
# 1. Add → Map → /local_costmap/costmap
# 2. Add → Image → /tof/depth/image_raw
# 3. Set Global Options → Frame: base_link
```

**Test Terrain Detection:**
```bash
# 1. Place rover on flat surface
# 2. Move rover toward obstacle (rock, step, wall)
# 3. Observe costmap updating in RViz
# 4. Costmap should show obstacles in red/yellow

# Monitor costmap updates
ros2 topic echo /local_costmap/costmap
# Should show cost values updating based on terrain
```

**Configuration Files Used:**
- `config/terrain_layer_params.yaml` — Terrain layer tuning
  - Height thresholds for obstacles
  - Cost scaling
  - Inflation radius

**Moon-Specific Parameters:**
```yaml
# From config/terrain_layer_params.yaml:
# Moon has loose regolith, need careful height thresholds

max_obstacle_height: 0.3   # Rocks > 30cm are obstacles
min_obstacle_height: 0.05  # Rocks < 5cm are traversable
height_cost_scaling_factor: 0.5  # Gentle scaling (not too aggressive)
traversability_threshold: 0.15   # Traversable if < 15cm variation
```

**Definition of Done:**
- [ ] Costmaps publishing
- [ ] Obstacles detected in costmap
- [ ] Height thresholds reasonable for moon terrain
- [ ] Ready for Phase 6

---

### **PHASE 6: Auto-Exposure Controller**

**File:** `phase_6_auto_exposure.launch.py`

**Purpose:** Dynamic camera exposure adjustment for harsh moon lighting (shadows and bright reflections).

**Nodes Launched:**
1. All previous nodes
2. `auto_exposure_controller_node` → adjusts camera parameters based on image brightness

**Adjusted Parameters:**
- `exposure_time` — Sensor integration time
- `analogue_gain` — Sensor amplification
- `digital_gain` — Post-processing amplification

**Launch Command:**
```bash
ros2 launch rover_launch phase_6_auto_exposure.launch.py
```

**Test Auto-Exposure:**
```bash
# 1. Point rover camera at bright light source (flashlight, window)
# 2. Observe video brightness in RViz adapts (brightens)
# 3. Point rover camera at dark area (shadow)
# 4. Observe video brightness adapts (darkens)
# 5. Video should remain viewable in both conditions

# Monitor auto-exposure adjustments
ros2 topic echo /camera_exposure_stats
# Should show: brightness value converging to target (128)
```

**Configuration Files Used:**
- `config/auto_exposure_params.yaml` — Exposure controller tuning

**Moon-Specific Parameters:**
```yaml
# From config/auto_exposure_params.yaml:
# Moon has harsh shadows (no atmosphere to diffuse light)

target_brightness: 128           # Target 50% brightness (neutral)
max_exposure_time_ms: 33         # 1/30 second (maintain frame rate)
max_analogue_gain: 4.0           # 4x sensor amplification max
max_digital_gain: 2.0            # 2x post-processing amplification
adjustment_rate_hz: 2.0          # Update every 0.5 seconds

# Moon-specific: aggressive adjustment due to extreme lighting
brightness_adjustment_step: 2    # 2% per update (faster)
convergence_threshold: 5         # Converge if within 5 brightness levels
```

**Definition of Done:**
- [ ] Auto-exposure adjusting exposure parameters
- [ ] Video visible in bright areas (no overexposure)
- [ ] Video visible in dark areas (no underexposure)
- [ ] Exposure parameters stable after convergence
- [ ] Ready for Phase 7

---

### **PHASE 7: Perception Pipeline**

**File:** `phase_7_perception.launch.py`

**Purpose:** On-device AI for terrain classification and landmark detection.

**Nodes Launched:**
1. All previous nodes
2. `imx500_classifier_node` → terrain/rock/crater classification
3. `yolo_detector_node` → landmark detection

**New Output:**
- `/ai_camera/classification` — Classification results (terrain type, confidence, bbox)
- `/landmarks/detections` — Detected landmarks with positions

**Launch Command:**
```bash
ros2 launch rover_launch phase_7_perception.launch.py
```

**Verify Perception:**
```bash
# Monitor classifications
ros2 topic echo /ai_camera/classification
# Should show classifications updating ~1-5 Hz
# Output format: label, confidence, bounding box

# Monitor landmark detections
ros2 topic echo /landmarks/detections
# Should show landmarks detected in view

# Visualize detections
# Terminal 2: Monitor camera with detections
ros2 run image_view image_view image:=/camera/image_raw
```

**Configuration Files Used:**
- IMX500 model (TensorFlow Lite) for classification
- YOLO model for landmark detection

**Moon-Specific Terrain Classes:**
```
- regolith (loose dust)
- bedrock (exposed rock)
- boulder (large rock)
- crater (depression)
- rover (self-detection for reference)
```

**Definition of Done:**
- [ ] Classifications publishing
- [ ] Landmark detections publishing
- [ ] Classification confidence > 60%
- [ ] At least 5 landmarks detected in typical view
- [ ] Ready for Phase 8

---

### **PHASE 8: Mission Health Monitor**

**File:** `phase_8_health_monitor.launch.py`

**Purpose:** Continuous system diagnostics to detect anomalies early.

**Nodes Launched:**
1. All previous nodes
2. `mission_health_monitor_node` → monitors all subsystems

**New Output:**
- `/mission_health` — Overall system health (OK, DEGRADED, CRITICAL)
- `/mission_events` — Events (anomalies detected)

**Health Metrics Monitored:**
- SLAM feature count (min 5)
- Odometry covariance (max 2.0)
- Loop closure rate (must detect loops)
- Landmark detection confidence (min 60%)
- Perception classification confidence (min 60%)

**Launch Command:**
```bash
ros2 launch rover_launch phase_8_health_monitor.launch.py
```

**Monitor Health:**
```bash
# Watch health status
ros2 topic echo /mission_health
# Should show:
# status: OK (or DEGRADED/CRITICAL if issues)
# slam_health: OK
# odometry_health: OK
# perception_health: OK

# Monitor events
ros2 topic echo /mission_events
# Should show events when thresholds crossed
```

**Definition of Done:**
- [ ] `/mission_health` publishing with OK status
- [ ] All subsystems reporting OK
- [ ] No DEGRADED or CRITICAL events (yet)
- [ ] Ready for Phase 9

---

### **PHASE 9: Nav2 with RTAB-Map**

**File:** `phase_9_nav2_rtabmap.launch.py`

**Purpose:** Full autonomous navigation using Nav2 stack with dynamic RTAB-Map.

**Nodes Launched:**
1. All previous nodes
2. `navigation2` stack:
   - AMCL localization
   - BT Navigator (behavior tree)
   - Controller Server (pure pursuit)
   - Planner Server (NavFN)
   - Recovery behaviors (spin, backup, wait)

**New Output:**
- `/tf` → localization transforms
- `/plan` — Path plan
- `/cmd_vel` — Velocity commands

**Launch Command:**
```bash
ros2 launch rover_launch phase_9_nav2_rtabmap.launch.py
```

**Test Navigation (Set Goals in RViz):**
```bash
# Terminal 1: Keep Phase 9 running
# Terminal 2: Launch RViz with Nav2 config
source ~/ros2_moon_rover/install/setup.bash
rviz2 -d ~/ros2_moon_rover/src/rover_launch/config/nav2.rviz 2>/dev/null || rviz2 &

# In RViz:
# 1. Set initial pose: 2D Pose Estimate button
# 2. Click on map where rover currently is
# 3. Set goal: Nav2 Goal button
# 4. Click on map where rover should go
# 5. Rover should plan and execute path

# Monitor navigation
ros2 topic echo /plan
# Should show path from rover to goal

# Monitor velocity commands
ros2 topic echo /cmd_vel
# Should show linear and angular velocities
```

**Configuration Files Used:**
- `config/nav2_params.yaml` — **200+ lines Nav2 tuning**
  - AMCL localization parameters
  - Path planner parameters
  - Controller parameters
  - Costmap parameters

**Moon-Specific Parameters:**
```yaml
# From config/nav2_params.yaml:
# Moon navigation needs conservative speeds due to rough terrain

amcl:
  max_particles: 500          # Localization particles
  min_particles: 200
  initial_pose_x: 0.0        # Start at origin
  initial_pose_y: 0.0

controller_server:
  max_vel_linear: 0.3         # 30 cm/s max (conservative for moon)
  max_vel_angular: 0.5        # 0.5 rad/s max
  controller_frequency: 10.0  # Hz

planner_server:
  expected_planner_frequency: 2.0  # Plan every 0.5 seconds
  max_planning_duration: 5.0       # Max 5 second planning time

local_costmap:
  plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
  inflation_radius: 0.5      # 50cm inflation around obstacles

global_costmap:
  plugins: ["static_layer", "inflation_layer"]
```

**Definition of Done:**
- [ ] Nav2 started successfully
- [ ] Can set initial pose in RViz
- [ ] Can set goals in RViz
- [ ] Rover plans paths
- [ ] Rover executes paths with recovery behaviors
- [ ] Navigation works in RTAB-Map for 5+ meters
- [ ] Ready for Phase 10

---

### **PHASE 10: Waypoint Recording & Playback**

**File:** `phase_10_waypoint_recording.launch.py`

**Purpose:** Record rover trajectories and replay them for mission automation.

**Nodes Launched:**
1. All previous nodes
2. `waypoint_recorder_node` → records poses to file
3. `waypoint_player_node` → plays back recorded waypoints

**Launch Command:**
```bash
ros2 launch rover_launch phase_10_waypoint_recording.launch.py
```

**Record a Waypoint Path:**
```bash
# Terminal 2: Start recording
source ~/ros2_moon_rover/install/setup.bash
ros2 service call /waypoint_recorder/start_recording std_srvs/SetBool "data: true"

# Manually drive rover around using joystick or Nav2 goals
# ... drive rover for 2-5 minutes ...

# Stop recording
ros2 service call /waypoint_recorder/stop_recording std_srvs/SetBool "data: false"

# Save waypoints to file
ros2 service call /waypoint_recorder/save_waypoints std_srvs/Trigger

# Check saved waypoints
cat /tmp/waypoints.yaml
# Should show list of recorded poses
```

**Playback Waypoints:**
```bash
# Terminal 3: Play back the path
source ~/ros2_moon_rover/install/setup.bash
ros2 service call /waypoint_player/load_waypoints path:"/tmp/waypoints.yaml" std_srvs/SetString
ros2 service call /waypoint_player/start_replay std_srvs/Trigger

# Rover should replay recorded trajectory
# Monitor progress
ros2 topic echo /waypoint_player/status
```

**Configuration Files Used:**
- `config/waypoint_params.yaml` — Recording parameters

**Moon-Specific Parameters:**
```yaml
# From config/waypoint_params.yaml:

recording:
  distance_threshold: 0.5    # Save waypoint every 50cm
  time_threshold: 5.0        # Or every 5 seconds
  angular_threshold: 0.2     # Or every 11.5 degree rotation

playback:
  goal_tolerance_linear: 0.2      # Reach goal within 20cm
  goal_tolerance_angular: 0.1     # Reach goal within 5.7 degrees
  max_speed: 0.3                  # 30 cm/s
```

**Definition of Done:**
- [ ] Waypoints recording successfully
- [ ] Waypoints saved to file
- [ ] Playback executed (rover follows recorded path)
- [ ] Playback accuracy reasonable (< 1m error)
- [ ] Ready for Phase 11

---

### **PHASE 11: Full Mission Integration**

**File:** `phase_11_full_mission.launch.py`

**Purpose:** All systems active — complete autonomous moon rover system ready for missions.

**Nodes Launched:**
1. All nodes from previous phases
2. `mission_logger_node` → records all data to rosbag
3. `streaming_node` → live MJPEG video server

**New Output:**
- Rosbag files in `/tmp/rover_missions/`
- MJPEG stream at `http://[pi-ip]:8080/stream`

**Launch Command:**
```bash
ros2 launch rover_launch phase_11_full_mission.launch.py
```

**Verify Full Mission System:**
```bash
# Monitor mission logging
ls -lh /tmp/rover_missions/
# Should show .db3 rosbag files being created

# Check mission events
ros2 topic echo /mission_health

# Monitor video stream
# On desktop PC or laptop:
# Open browser: http://[pi-ip]:8080/stream
# Should see live rover camera feed in MJPEG format

# Test mission execution (30 minute autonomous mission):
# 1. Record waypoint path (Phase 10)
# 2. Play back waypoints (Phase 10)
# 3. Monitor health (Phase 8)
# 4. Log all data (Phase 11)

# After mission complete, analyze:
# Terminal 2:
cd /tmp/rover_missions
# Use ROS 2 tools to analyze bags:
ros2 bag info *.db3
```

**Configuration Files Used:**
- All previous configuration files
- `config/mission_params.yaml` — Mission-wide settings

**Moon-Specific Mission Profile:**
```yaml
# From mission parameters:

mission_duration: 1800  # 30 minutes max
battery_warning_level: 20  # Warn at 20% battery
communication_timeout: 60  # 60 second comm check
terrain_difficulty: moderate  # Regolith with scattered rocks
expected_speed: 0.2  # 20 cm/s average
expected_coverage: 1500  # meters of traversal
```

**Definition of Done:**
- [ ] All previous phases functioning
- [ ] Mission logger recording data
- [ ] Video streaming accessible
- [ ] 30-minute mission completed without errors
- [ ] All data logged for analysis
- [ ] **System ready for deployment!**

---

## 🚀 SEQUENTIAL TESTING CHECKLIST

**Follow this sequence. Do NOT skip phases!**

- [ ] **Phase 0** — Hardware check (all devices found)
- [ ] **Phase 1** — Sensors (4 topics publishing at correct rates)
- [ ] **Phase 2** — TF tree (3 sensor frames visible)
- [ ] **Phase 3** — EKF (square drive test passes)
- [ ] **Phase 4** — SLAM (loop closure detected, map quality good)
- [ ] **Phase 5** — Terrain layer (costmap shows obstacles)
- [ ] **Phase 6** — Auto-exposure (video visible in bright and dark)
- [ ] **Phase 7** — Perception (classifications and detections working)
- [ ] **Phase 8** — Health monitor (all subsystems OK)
- [ ] **Phase 9** — Nav2 (autonomous navigation working, 5+ meters)
- [ ] **Phase 10** — Waypoint recording (paths recorded and played back)
- [ ] **Phase 11** — Full mission (30-minute mission with logging)

---

## 📊 EXPECTED LAUNCH TIMES

| Phase | Startup Time | Status |
|-------|--------------|--------|
| Phase 0 | 2 seconds | Hardware check |
| Phase 1 | 5 seconds | Sensor startup |
| Phase 2 | 1 second | TF publishing |
| Phase 3 | 3 seconds | EKF startup |
| Phase 4 | 5 seconds | SLAM startup |
| Phase 5 | 3 seconds | Costmap startup |
| Phase 6 | 1 second | Auto-exposure startup |
| Phase 7 | 5 seconds | Perception startup |
| Phase 8 | 2 seconds | Health monitor startup |
| Phase 9 | 10 seconds | Nav2 full stack |
| Phase 10 | 5 seconds | Waypoint recording |
| Phase 11 | 15 seconds | Full mission |

---

## 🆘 EMERGENCY LAUNCH TROUBLESHOOTING

### Phase Fails to Launch
```bash
# Check ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/ros2_moon_rover/install/setup.bash

# Verify package exists
ros2 pkg list | grep rover_launch

# Try with verbose output
ros2 launch rover_launch phase_X_*.launch.py --verbose
```

### "Package not found"
```bash
# Make sure workspace is built
cd ~/ros2_moon_rover
colcon build --symlink-install

# Make sure workspace is sourced
source install/setup.bash
```

### "Could not find launch file"
```bash
# Check launch file exists
ls ~/ros2_moon_rover/src/rover_launch/launch/ | grep phase_

# Make sure filename matches exactly (case-sensitive)
ros2 launch rover_launch phase_1_sensors.launch.py  # Correct
ros2 launch rover_launch phase_1_Sensors.launch.py  # Wrong!
```

### Configuration File Not Found
```bash
# Check config directory structure
ls ~/ros2_moon_rover/src/rover_launch/config/

# Make sure launch file references correct path
# Should use: $(find rover_launch)/config/[filename].yaml
```

---

## 📋 CONFIGURATION FILE VERIFICATION

Before running phases, verify all configuration files exist:

```bash
# Essential configuration files
ls ~/ros2_moon_rover/src/rover_launch/config/
```

**Must include:**
- `ekf_params.yaml` — EKF tuning (Phase 3)
- `rtabmap_params.yaml` — SLAM tuning (Phase 4)
- `nav2_params.yaml` — Nav2 tuning (Phase 9)
- `static_transforms.yaml` — TF tree (Phase 2)
- `imx500_intrinsics.yaml` — Camera calibration (Phase 1)
- `tof_intrinsics.yaml` — Depth camera calibration (Phase 1)
- `imu_calibration.yaml` — IMU offsets (Phase 1)
- `auto_exposure_params.yaml` — Auto-exposure tuning (Phase 6)
- `terrain_layer_params.yaml` — Terrain detection (Phase 5)

**If any missing:**
- Copy from corresponding implementation document
- Or from 11_CONFIGURATION_REFERENCE.md

---

**All phases tested? → System ready for real moon rover deployment! 🌙🚀**
