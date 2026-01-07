# 06_EKF_FUSION_NODE.md - Complete EKF Implementation
# Place this in: src/ekf_fusion_node/

## FILE STRUCTURE:
```
ekf_fusion_node/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── ekf_node.cpp
├── config/
│   └── ekf_params.yaml
└── launch/
    └── ekf.launch.py
```

---

## FILE 1: ekf_fusion_node/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>ekf_fusion_node</name>
  <version>0.0.1</version>
  <description>EKF odometry fusion of VEX encoders + IMU using robot_localization</description>
  <maintainer email="rover@moonbase.local">Rover Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>rclcpp</build_depend>
  <build_depend>robot_localization</build_depend>
  <build_depend>nav_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>tf2_ros</build_depend>
  <build_depend>geometry_msgs</build_depend>

  <exec_depend>rclcpp</exec_depend>
  <exec_depend>robot_localization</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## FILE 2: ekf_fusion_node/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(ekf_fusion_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_localization REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Note: robot_localization provides ekf_node executable
# This package is a wrapper that provides configuration

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

---

## FILE 3: ekf_fusion_node/config/ekf_params.yaml

```yaml
# Extended Kalman Filter (EKF) Configuration for Moon Rover
# Fuses wheel odometry + IMU using robot_localization
# https://docs.ros.org/en/humble/p/robot_localization/

ekf_filter_node:
  ros__parameters:
    # ============================================================================
    # INPUT SOURCES
    # ============================================================================
    
    # Odometry from VEX wheel encoders
    odom0: /vex/odom_raw
    odom0_config: [true,    # X position
                   true,    # Y position
                   false,   # Z position (ignore - rover is 2D on flat terrain)
                   false,   # Roll (ignore - should be 0)
                   false,   # Pitch (ignore - should be 0)
                   true,    # Yaw heading
                   true,    # X velocity
                   true,    # Y velocity
                   false,   # Z velocity
                   false,   # Roll rate
                   false,   # Pitch rate
                   true,    # Yaw rate
                   false,   # X acceleration
                   false,   # Y acceleration
                   false]   # Z acceleration
    odom0_relative: false   # odom0 is in world frame (not relative)
    odom0_queue_size: 10
    odom0_nodelay: false
    odom0_differential: false

    # IMU (accelerometer + gyroscope, no magnetometer)
    imu0: /imu/data
    imu0_config: [false,   # X position
                  false,   # Y position
                  false,   # Z position
                  true,    # Roll (use gyro roll rate for attitude estimation)
                  true,    # Pitch (use gyro pitch rate)
                  false,   # Yaw (encoder provides better yaw)
                  false,   # X velocity
                  false,   # Y velocity
                  false,   # Z velocity
                  true,    # Roll rate (direct from gyro)
                  true,    # Pitch rate (direct from gyro)
                  false,   # Yaw rate (encoder-derived is better)
                  true,    # X acceleration (accel for tilt sensing)
                  true,    # Y acceleration
                  true]    # Z acceleration
    imu0_relative: true    # IMU rates are relative (not absolute)
    imu0_queue_size: 10
    imu0_nodelay: false
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true  # Remove gravity from accel

    # ============================================================================
    # FILTER STATE & DYNAMICS
    # ============================================================================
    
    frequency: 50.0                    # EKF update rate (Hz)
    two_d_mode: false                  # Allow Z but don't use it
    publish_acceleration: false
    
    # Initial state covariance
    # Dimensions: [X, Y, Z, Roll, Pitch, Yaw, VX, VY, VZ, WRoll, WPitch, WYaw, AX, AY, AZ]
    initial_estimate_covariance: [1.0,     # X position (1 m uncertainty)
                                  1.0,     # Y position
                                  1000.0,  # Z (don't use)
                                  1000.0,  # Roll (don't use)
                                  1000.0,  # Pitch (don't use)
                                  1.0,     # Yaw (0.1 rad = 5.7 degrees)
                                  1.0,     # VX velocity
                                  1.0,     # VY velocity
                                  1000.0,  # VZ (don't use)
                                  1000.0,  # WRoll (don't use)
                                  1000.0,  # WPitch (don't use)
                                  1.0,     # WYaw angular velocity
                                  1000.0,  # AX (don't use)
                                  1000.0,  # AY (don't use)
                                  1000.0]  # AZ (don't use)

    # ============================================================================
    # PROCESS NOISE (Q matrix) - How much we trust model predictions
    # ============================================================================
    # Lower values = trust model more = less correction
    # Higher values = trust measurements more = faster adaptation
    
    process_noise_p_p_x: 0.03           # Position process noise (m^2)
    process_noise_p_p_y: 0.03
    process_noise_p_p_z: 999.0          # Don't estimate Z
    process_noise_p_r_r: 999.0          # Don't estimate Roll
    process_noise_p_r_p: 999.0          # Don't estimate Pitch
    process_noise_p_r_y: 0.03           # Yaw drift on flat terrain is minimal
    
    process_noise_p_v_x: 0.1            # Velocity process noise (m^2/s^2)
    process_noise_p_v_y: 0.1
    process_noise_p_v_z: 999.0
    
    process_noise_p_w_x: 999.0          # Angular velocity process noise
    process_noise_p_w_y: 999.0
    process_noise_p_w_z: 0.02           # Yaw rate drift minimal
    
    process_noise_p_a_x: 999.0          # Acceleration process noise (don't estimate)
    process_noise_p_a_y: 999.0
    process_noise_p_a_z: 999.0

    # ============================================================================
    # MEASUREMENT NOISE (R matrix) - How much we trust sensor measurements
    # ============================================================================
    # From /vex/odom_raw covariance
    odom0_pose_p_p_x: 0.03              # Encoder position uncertainty
    odom0_pose_p_p_y: 0.03
    odom0_pose_p_p_z: 999.0
    odom0_pose_p_r_r: 999.0
    odom0_pose_p_r_p: 999.0
    odom0_pose_p_r_y: 0.05              # Yaw uncertainty from encoders
    
    odom0_twist_p_v_x: 0.05             # Encoder velocity uncertainty
    odom0_twist_p_v_y: 0.05
    odom0_twist_p_v_z: 999.0
    odom0_twist_p_w_x: 999.0
    odom0_twist_p_w_y: 999.0
    odom0_twist_p_w_z: 0.1              # Yaw rate uncertainty

    # From /imu/data covariance
    imu0_pose_p_r_r: 0.3                # IMU roll uncertainty (radians^2)
    imu0_pose_p_r_p: 0.3                # IMU pitch uncertainty
    
    imu0_twist_p_w_x: 0.05              # Gyro roll rate uncertainty
    imu0_twist_p_w_y: 0.05              # Gyro pitch rate uncertainty
    
    imu0_linear_a_p_a_x: 0.5            # Accelerometer uncertainty (m^2/s^4)
    imu0_linear_a_p_a_y: 0.5

    # ============================================================================
    # FRAME IDS & TF PUBLISHING
    # ============================================================================
    
    map_frame: map                       # World frame
    odom_frame: odom                     # Odometry frame
    base_link_frame: base_link           # Robot frame
    world_frame: map
    
    print_diagnostics: true              # Log filter status
    publish_tf: true                     # Broadcast odom -> base_link TF

    # ============================================================================
    # TUNING GUIDELINES FOR MOON TERRAIN
    # ============================================================================
    # 
    # Low-texture terrain characteristics:
    # - Minimal visual odometry drift over 10-20 meter runs
    # - Wheel slip is primary error source (sandy/loose soil)
    # - IMU roll/pitch dominated by gravity (useful for tilt sensing)
    # - Gyro bias drift is small over mission duration (<5 minutes)
    #
    # Tuning procedure:
    # 1. Test with high encoder process noise (0.05) - filter trusts encoders
    # 2. Drive rover in square: 5m forward, 90° turn, repeat 4x
    # 3. Check final position: should return close to start (<0.5m error)
    # 4. If drift too large: DECREASE process_noise_p_p_* (trust model less)
    # 5. If estimate too noisy: INCREASE process_noise_p_p_* (trust model more)
    #
    # Expected performance:
    # - Odometry drift: <0.5 m per 10 m traverse
    # - Yaw drift: <10 degrees per 10 m traverse (with encoder)
    # - Update frequency: 50 Hz (20ms latency)
    # - CPU usage: <5% on Pi5
```

---

## FILE 4: ekf_fusion_node/launch/ekf.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('ekf_fusion_node')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'ekf_params.yaml'])
    
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[config_file],
            remappings=[
                ('odometry/filtered', '/odom'),
                ('accel/filtered', '/acceleration_filtered'),
            ],
            output='screen'
        ),
    ])
```

---

## TESTING THE EKF

```bash
# Build
cd ~/ros2_moon_rover
colcon build --packages-select ekf_fusion_node --symlink-install

# Test (requires Phase 1 sensors running)
# Terminal 1: Launch sensors + TF
ros2 launch rover_launch phase_1_sensors.launch.py &
ros2 run static_tf_publisher static_tf_broadcaster &

# Terminal 2: Launch EKF
ros2 launch ekf_fusion_node ekf.launch.py

# Terminal 3: Monitor
source ~/ros2_moon_rover/install/setup.bash
ros2 topic echo /odom

# Expected:
# - /odom published at 50 Hz
# - position.x, position.y increase as rover moves
# - orientation.z (yaw) changes on turns
# - pose_covariance should be relatively small and stable
```

---

## CALIBRATION & TUNING

### Step 1: Verify Encoder Calibration
```bash
# Check wheel_radius and wheel_separation in vex_params.yaml
# Walk rover 1 meter forward, check /odom.pose.pose.position.x
# Should read 1.0 ± 0.1 m
# If off, adjust wheel_radius
```

### Step 2: Verify IMU Calibration
```bash
# Stationary IMU should show:
# ros2 topic echo /imu/data
# linear_acceleration.z = -9.81 (gravity)
# angular_velocity = 0
# If not, calibrate offsets in imu_params.yaml
```

### Step 3: EKF Tuning
```bash
# Record a square traverse
ros2 bag record /odom /vex/odom_raw /imu/data &
# Move rover in square: forward 5m, turn 90°, repeat 4x
# Final position should be near start

# If drifting too much: DECREASE process_noise_p_p_x/y
# If estimate too noisy: INCREASE process_noise_p_p_x/y
```

### Step 4: Monitor Covariance
```bash
# Healthy covariance trace
ros2 topic echo /odom | grep -A 3 "pose_covariance"
# x variance (index 0) should be ~0.01-0.1
# y variance (index 7) should be ~0.01-0.1
# yaw variance (index 35) should be ~0.02-0.05
```

---

## PERFORMANCE TARGETS

| Metric | Target | Acceptable |
|--------|--------|-----------|
| Odometry update rate | 50 Hz | 30 Hz minimum |
| Position drift | <0.5 m / 10 m traverse | <1 m / 10 m |
| Yaw drift | <10° / 10 m traverse | <20° / 10 m |
| Filter convergence | <5 seconds | <10 seconds |
| CPU usage | <5% | <10% |

---

## CRITICAL NOTES

1. **EKF must start AFTER sensors** — Otherwise tf lookup errors occur
2. **Covariance must be reasonable** — If too high (>1.0), sensors may be miscalibrated
3. **Odom frame is relative** — TF chain should be: map → odom → base_link
4. **Two 2D operation** — Set `two_d_mode: false` but ignore Z/Roll/Pitch for rover

---

## NEXT: Perception nodes

Request **07_PERCEPTION_NODES.md** for classifiers, detectors, and auto-exposure controller.
