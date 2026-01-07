# 11_CONFIGURATION_REFERENCE.md - Complete YAML Configs & Tuning Guides
# Central repository of all configuration files for the ROS 2 moon rover

---

# TABLE OF CONTENTS

1. **RTAB-Map Configuration** (SLAM tuning for low-texture terrain)
2. **Nav2 Configuration** (Navigation stack parameters)
3. **Camera Calibration** (Intrinsics, extrinsics, static transforms)
4. **Sensor Calibration** (IMU offsets, VEX encoder calibration)
5. **Terrain Layer Configuration** (Costmap plugin for ToF)
6. **Hardware Configuration** (Serial ports, I2C addresses)

---

# 1. RTAB-MAP CONFIGURATION FOR MOON TERRAIN

## Location: src/rover_launch/config/rtabmap_params.yaml

```yaml
# RTAB-Map (Real-Time Appearance-Based Mapping) Configuration
# Optimized for low-texture moon terrain
# Reference: http://wiki.ros.org/rtabmap

rtabmap:
  ros__parameters:
    # ========================================================================
    # GENERAL PARAMETERS
    # ========================================================================
    
    verbose: true
    use_sim_time: false
    pub_tf: true
    tf_delay: 0.05
    
    # ========================================================================
    # INPUT/OUTPUT
    # ========================================================================
    
    subscribe_depth: true            # Use depth camera
    subscribe_odom: true             # Use /odom
    subscribe_scan: false            # No 2D laser scanner
    subscribe_scan_cloud: false
    odom_frame_id: odom
    approx_sync: true                # Sync RGB + depth
    approx_sync_max_interval: 0.05   # 50ms tolerance
    
    # ========================================================================
    # FEATURE DETECTION & EXTRACTION
    # ========================================================================
    # For low-texture terrain, we need aggressive feature detection
    
    Rtabmap/VsrmInliers: 200          # Visual odometry: inlier threshold (lower=more features used)
    Rtabmap/VsrmReprojThr: 0.5        # Reprojection error threshold (pixels)
    
    # Feature detector: BRIEF (fast) or SIFT (more robust but slower)
    Rtabmap/VsrmStrategy: 0            # 0=epipolar, 1=fundamental (use 0 for speed)
    
    # Keyframe parameters
    Rtabmap/TimeThreshold: 1000        # Min time between keyframes (ms)
    Rtabmap/DistanceThreshold: 0.1     # Min distance between keyframes (m)
    
    # Feature extraction
    Rtabmap/VsrmMaxFeatures: 200       # Max ORB features per image
    Rtabmap/VsrmMinInliers: 20         # Min inliers to accept visual odometry (low for moon terrain)
    Rtabmap/VsrmMaxDepthDistance: 5.0  # Max depth to consider (5m for moon terrain)
    
    # ========================================================================
    # LOOP CLOSURE DETECTION
    # ========================================================================
    # Critical for long missions on featureless terrain
    
    Rtabmap/LoopThr: 0.11              # Loop closure threshold (lower=more sensitive, 0.11 typical)
    Rtabmap/LoopRatio: 0.6             # Similarity ratio requirement
    Rtabmap/TimeThr: 0                 # Time threshold (0=disabled, allow loop closure anytime)
    
    # Loop closure detection frequency
    Rtabmap/DetectorStrategy: 0        # 0=SIFT (most robust), others faster
    
    # ========================================================================
    # MAPPING & OPTIMIZATION
    # ========================================================================
    
    Rtabmap/CreateOccupancyGrid: true
    Rtabmap/GridSize: 0.05             # Occupancy grid cell size (5cm)
    
    # Graph optimization
    Rtabmap/GraphOptimizer: 2          # 0=GTSAM, 1=g2o, 2=Ceres (use Ceres for robustness)
    Rtabmap/GraphStrategy: 0           # 0=optimized graph (use this)
    
    # ========================================================================
    # RGBD VISUAL ODOMETRY (for VEX odometry fallback)
    # ========================================================================
    
    OdomF2M/MaxSize: 500               # Max features in frame-to-model
    OdomF2M/MinInliers: 10             # Min inliers to accept odometry
    OdomF2M/MaxDepthDistance: 5.0
    OdomF2M/TimeThreshold: 500         # Update frequency
    
    # ========================================================================
    # DEPTH SENSOR PARAMETERS
    # ========================================================================
    
    depth_decimation: 4                # Reduce depth resolution (4x faster)
    depth_roofline: 5.0                # Max depth (5m for moon)
    depth_noise_sigma: 0.05            # Depth noise estimate
    cloud_decimation: 4
    
    # ========================================================================
    # PERFORMANCE TUNING
    # ========================================================================
    
    # Working memory
    Rtabmap/MemoryStrategy: 0          # 0=keep newest in memory
    Rtabmap/MaxNodes: 2000             # Max nodes in memory
    
    # Database optimization
    Rtabmap/DatabaseCleanupThreshold: 1000
    Rtabmap/LoopClosureIdentityGuess: false
    
    # Memory usage
    Rtabmap/ImageDecimation: 1         # Keep full resolution
    Rtabmap/ImagePostProcess: 0        # No post-processing
    Rtabmap/ImageNormalized: false     # Don't normalize (may help with exposure changes)
    
    # ========================================================================
    # TUNING FOR LOW-TEXTURE MOON TERRAIN
    # ========================================================================
    #
    # Moon terrain is: gray, low-contrast, sparse features, repetitive
    # Challenges: feature dropout in shadows, loop closure ambiguity
    #
    # Strategy:
    # 1. Rely on VEX wheel odometry (primary, most accurate)
    # 2. Use visual odometry as fallback/validation
    # 3. Aggressive loop closure (even low confidence helps with long missions)
    # 4. Keep feature threshold LOW to catch sparse features
    # 5. Use IMU for roll/pitch estimation (gravity helps on flat terrain)
    #
    # Performance targets:
    # - Visual odometry drift: < 5% (mission time * velocity)
    # - Loop closure rate: >50% of loop opportunities detected
    # - Update latency: <100ms
    # - CPU usage: <30% on Pi5
    #
    # Tuning procedure:
    # 1. Record a mission bag
    # 2. Replay with different parameters
    # 3. Compare /rtabmap/mapData with ground truth (if available)
    # 4. Adjust VsrmMinInliers / DistanceThreshold for better tracking
    # 5. Adjust LoopThr / DetectorStrategy for loop closure
    #
    # Common issues:
    # - "Not enough inliers" (VsrmMinInliers too high) → DECREASE it
    # - "Drifting" (features unreliable) → INCREASE DistanceThreshold
    # - "Loops not detected" (LoopThr too strict) → DECREASE LoopThr
    # - "Too many false positives" (LoopThr too loose) → INCREASE LoopThr
    
    # Conservative settings (most robust for moon terrain)
    Rtabmap/VsrmMinInliers: 15         # Allow drift in exchange for robustness
    Rtabmap/LoopThr: 0.08              # Detect many loop closures (even weak ones help)
    
rtabmap_ros:
  ros__parameters:
    visual_odometry: true              # Enable visual odometry
    odom_frame_id: odom
    map_frame_id: map
```

---

# 2. NAV2 CONFIGURATION FOR MOON ROVER

## Location: src/rover_launch/config/nav2_params.yaml

```yaml
# Nav2 (Navigation2) Configuration
# For moon rover with ToF-based terrain layer

amcl:
  ros__parameters:
    # AMCL (Monte Carlo Localization) - may not be needed if using RTAB-Map
    use_sim_time: false
    alpha1: 0.2                        # Rotation noise (particles)
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: base_link
    global_frame_id: map
    odom_frame_id: odom
    z_hit: 0.5
    z_short: 0.05
    z_max: 0.05
    z_rand: 0.5
    sigma_hit: 0.2

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_bt_xml_filename: nav2_tree.xml
    enable_groot_monitoring: false
    enable_groot_debug_terminal: false

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    
    progress_checker:
      plugin: nav2_core::SimpleProgressChecker
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    
    general_goal_checker:
      stateful: True
      plugin: nav2_core::SimpleGoalChecker
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
    
    FollowPath:
      plugin: nav2_pure_pursuit_controller::PurePursuitController
      desired_linear_vel: 0.2            # 0.2 m/s (slow, safe for moon)
      lookahead_dist: 0.325
      max_angular_vel: 1.0
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      max_allowable_linear_vel: 0.3
      min_field_of_view: 4.712            # 270 degrees
      use_collision_detection: false
      transform_tolerance: 0.1
      regulated_linear_scaling_min_speed: 0.10
      regulated_linear_scaling_max_speed: 0.26
      regulated_linear_scaling_stop_distance: 0.9

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: nav2_navfn_planner::NavfnPlanner
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: nav2_recoveries::Spin
      max_rotations: 1.0
      max_effort: 320
      cmd_vel: cmd_vel
      plugin_lib_names:
        - nav2_recoveries_plugins
    backup:
      plugin: nav2_recoveries::BackUp
      max_translational_speed: 0.15
      min_translational_speed: -0.15
      max_rotational_speed: 1.57
      min_rotational_speed: -1.57
      max_distance: 0.15
      plugin_lib_names:
        - nav2_recoveries_plugins
    wait:
      plugin: nav2_recoveries::Wait
      wait_duration: 2
      plugin_lib_names:
        - nav2_recoveries_plugins

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.175
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        cost_scaling_factor: 10.0
        inflation_radius: 0.55
      obstacle_layer:
        plugin: nav2_costmap_2d::ObstacleLayer
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: LaserScan
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_subscribe_transient_local: True
      always_send_full_costmap: True
  local_costmap_client:
    ros__parameters:
      use_sim_time: false
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: false

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      robot_radius: 0.175
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: nav2_costmap_2d::ObstacleLayer
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: LaserScan
      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        cost_scaling_factor: 10.0
        inflation_radius: 0.55
      always_send_full_costmap: True
  global_costmap_client:
    ros__parameters:
      use_sim_time: false
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: false
```

---

# 3. CAMERA CALIBRATION FILES

## Location: src/rover_launch/config/imx500_intrinsics.yaml

```yaml
# IMX500 Camera Intrinsic Calibration
# Generated from: rosrun camera_calibration cameracalibrator.py --size 8x6 ...
# Date: 2026-01-06

camera_matrix:
  rows: 3
  cols: 3
  data: [
    1234.56,    0.0,  640.5,
       0.0, 1234.56,  360.5,
       0.0,    0.0,    1.0
  ]

dist_coeffs:
  rows: 1
  cols: 5
  data: [-0.25, 0.05, 0.0, 0.0, 0.0]

image_width: 1280
image_height: 720
camera_name: imx500

# NOTE: These are example values. Run camera calibration before deployment!
# Command:
#   rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 \
#     image:=/camera/image_raw camera:=/camera
```

## Location: src/rover_launch/config/tof_intrinsics.yaml

```yaml
# Arducam ToF Camera Intrinsic Calibration

camera_matrix:
  rows: 3
  cols: 3
  data: [
    350.0,   0.0, 160.0,
      0.0, 350.0, 120.0,
      0.0,   0.0,   1.0
  ]

dist_coeffs:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]

image_width: 320
image_height: 240
camera_name: tof_camera

# Depth range
min_depth_m: 0.1
max_depth_m: 4.0

# NOTE: These are nominal values. Calibrate with checkerboard for accuracy.
```

## Location: src/rover_launch/config/static_transforms.yaml

```yaml
# Static Transform Tree for Moon Rover
# Defines fixed relationships between sensor frames

static_transforms:
  # Base link to camera (RGB/AI)
  - parent_frame: base_link
    child_frame: camera_link
    translation:
      x: 0.20      # 20cm forward
      y: 0.0       # centered
      z: 0.10      # 10cm up
    rotation:
      roll: 0.0
      pitch: 0.0
      yaw: 0.0

  # Base link to ToF camera
  - parent_frame: base_link
    child_frame: tof_link
    translation:
      x: 0.20      # 20cm forward (same as RGB)
      y: 0.05      # 5cm right (offset for stereo effect)
      z: 0.10      # 10cm up
    rotation:
      roll: 0.0
      pitch: 0.0
      yaw: 0.0

  # Base link to IMU
  - parent_frame: base_link
    child_frame: imu_link
    translation:
      x: 0.0       # centered
      y: 0.0       # centered
      z: 0.05      # 5cm up
    rotation:
      roll: 0.0
      pitch: 0.0
      yaw: 0.0

# CALIBRATION PROCEDURE:
# 1. Place rover on flat surface
# 2. Measure distances from base_link (center of axle) to each sensor
# 3. Use RViz to validate TF tree
# 4. Project ToF points into RGB image - should align with features
```

---

# 4. SENSOR CALIBRATION PARAMETERS

## Location: src/rover_launch/config/imu_calibration.yaml

```yaml
# MPU6050 IMU Calibration Offsets
# Calibrated on stationary rover

accelerometer_offsets:
  x: -0.05     # m/s^2
  y: 0.02
  z: 0.10

gyroscope_offsets:
  x: -0.01     # rad/s
  y: 0.02
  z: -0.005

# Calibration procedure:
# 1. Place rover on flat, level surface
# 2. Record /imu/data for 30 seconds while stationary
# 3. Calculate mean acceleration (should be near [0, 0, 9.81])
# 4. Calculate mean gyro (should be near [0, 0, 0])
# 5. Subtract means from raw values
#
# Recalibrate if:
# - Gyro drift > 0.05 rad/s after 1 hour
# - Accel changes after thermal equilibration
# - Temperature changes significantly (moon: -150 to +120°C)
```

## Location: src/rover_launch/config/vex_encoder_calibration.yaml

```yaml
# VEX Robotics V5 Wheel Encoder Calibration

# Wheel geometry
wheel_radius_m: 0.047     # 47mm wheels (measure actual)
wheel_separation_m: 0.35  # distance between left/right wheels (measure actual)
encoder_resolution: 360   # counts per revolution (V5 : 360 CPR)

# Calibration procedure:
# 1. Measure actual wheel radius: π * diameter / 2
# 2. Drive forward 1.0 meter, check /odom.pose.pose.position.x
#    - If reading < 1.0: wheel_radius too small (increase)
#    - If reading > 1.0: wheel_radius too large (decrease)
# 3. Perform 3-point calibration: forward, left turn, right turn
# 4. Check that return position ≈ start position

# Encoder counts
left_encoder_offset: 0     # For encoder non-alignment
right_encoder_offset: 0

# Drive characterization (optional)
left_motor_scale: 1.0      # Adjust if left/right motors have different power
right_motor_scale: 1.0
```

---

# 5. TERRAIN LAYER CONFIGURATION

## Location: src/terrain_layer_plugin/config/terrain_layer_params.yaml

```yaml
# ToF-based Terrain Layer for Nav2 Costmap
# Converts depth data to traversability layer

terrain_layer:
  # Layer properties
  enabled: true
  plugin: terrain_layer_plugin::TerrainLayer
  
  # Depth source
  depth_topic: /tof/depth/image_raw
  depth_info_topic: /camera/info_tof
  
  # Terrain classification
  max_height_mm: 100       # Obstacles > 10cm are non-traversable
  min_height_mm: 0
  step_height_mm: 50       # Ledges > 5cm cost high
  
  # Costmap encoding
  cost_no_obstacle: 0
  cost_low_obstacle: 128   # Step-sized obstacles
  cost_obstacle: 254       # Non-traversable
  
  # Update frequency
  update_frequency: 2.0    # Hz
  observation_persistence: 2.0  # Keep observations for 2 seconds
```

---

# BUILD INSTRUCTIONS WITH CONFIGS

```bash
# Copy all YAML files to src/rover_launch/config/
mkdir -p ~/ros2_moon_rover/src/rover_launch/config

# Copy from this document
cp rtabmap_params.yaml ~/ros2_moon_rover/src/rover_launch/config/
cp nav2_params.yaml ~/ros2_moon_rover/src/rover_launch/config/
cp imx500_intrinsics.yaml ~/ros2_moon_rover/src/rover_launch/config/
cp tof_intrinsics.yaml ~/ros2_moon_rover/src/rover_launch/config/
cp static_transforms.yaml ~/ros2_moon_rover/src/rover_launch/config/
cp imu_calibration.yaml ~/ros2_moon_rover/src/rover_launch/config/
cp vex_encoder_calibration.yaml ~/ros2_moon_rover/src/rover_launch/config/
cp terrain_layer_params.yaml ~/ros2_moon_rover/src/rover_launch/config/

# Build with configs
cd ~/ros2_moon_rover
colcon build --symlink-install
```

---

## NEXT STEPS

1. Review and customize all YAML files for your hardware
2. Run camera/IMU calibration procedures
3. Test each component with proper configs
4. Refer back to this file when tuning performance

All configs are documented with **TUNING GUIDELINES** specific to moon terrain!
