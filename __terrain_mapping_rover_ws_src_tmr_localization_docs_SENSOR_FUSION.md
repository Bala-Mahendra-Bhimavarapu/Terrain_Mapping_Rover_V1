# Sensor Fusion Architecture

## Overview

The TMR localization system uses an Extended Kalman Filter (EKF) to fuse multiple sensor inputs into a single, more accurate pose estimate.

## Sensor Inputs

### 1. Wheel Odometry (`/vex/odom_raw`)

**Source**: `vex_serial_node`
**Message Type**: `nav_msgs/Odometry`
**Rate**: 50 Hz

**Provides**:
- Position (x, y)
- Orientation (yaw)
- Linear velocity (vx)
- Angular velocity (vyaw)

**Strengths**:
- Good short-term accuracy
- Direct measurement of wheel motion
- No drift during stationary periods

**Weaknesses**:
- Accumulates error over time (drift)
- Affected by wheel slip
- No absolute reference

### 2. IMU (`/imu/data`)

**Source**: `imu_node` (MPU6050)
**Message Type**: `sensor_msgs/Imu`
**Rate**: 100 Hz

**Provides**: 
- Orientation (from complementary filter)
- Angular velocity (gyroscope)
- Linear acceleration (accelerometer)

**Strengths**:
- High update rate
- Not affected by wheel slip
- Good for detecting rotation

**Weaknesses**: 
- Gyro drift over time
- Accelerometer noise
- Cannot provide position directly

## Fusion Strategy

### What We Fuse

```
Wheel Odometry: 
  ✓ Position X      (primary source)
  ✓ Position Y      (primary source)
  ✗ Position Z      (2D robot)
  ✗ Roll            (2D robot)
  ✗ Pitch           (2D robot)
  ✓ Yaw             (combined with IMU)
  ✓ Velocity X      (primary source)
  ✗ Velocity Y      (differential drive)
  ✗ Velocity Z      (2D robot)
  ✗ Angular Vel X   (2D robot)
  ✗ Angular Vel Y   (2D robot)
  ✓ Angular Vel Z   (combined with IMU)

IMU:
  ✗ Position        (not provided)
  ✗ Roll            (2D mode)
  ✗ Pitch           (2D mode)
  ✓ Yaw             (combined with odom)
  ✗ Velocity        (not provided)
  ✗ Angular Vel X   (2D mode)
  ✗ Angular Vel Y   (2D mode)
  ✓ Angular Vel Z   (yaw rate)
  ✓ Accel X         (forward acceleration)
  ✓ Accel Y         (lateral acceleration)
  ✗ Accel Z         (gravity, not useful for 2D)
```

### Why This Configuration? 

1. **Position (X, Y)**: Only wheel odometry provides position.  The IMU's accelerometer could theoretically be double-integrated, but this accumulates massive drift.

2. **Yaw**: Both sensors provide yaw. The EKF combines them: 
   - Wheel odometry:  Good for slow changes, affected by slip
   - IMU: Good for fast changes, drifts over time
   - Combined: Best of both worlds

3. **Yaw Rate**: Both sensors provide angular velocity: 
   - Wheel odometry:  Calculated from wheel speeds
   - IMU gyroscope: Direct measurement
   - Combined: Reduces noise and improves accuracy

4. **Acceleration**: IMU provides forward/lateral acceleration, which helps the filter predict velocity changes.

## Data Flow

```
┌─────────────────┐     ┌─────────────────┐
│  vex_serial_    │     │    imu_node     │
│     node        │     │   (MPU6050)     │
└────────┬────────┘     └────────┬────────┘
         │                       │
         │ /vex/odom_raw         │ /imu/data
         │ (50 Hz)               │ (100 Hz)
         │                       │
         └───────────┬───────────┘
                     │
                     ▼
         ┌───────────────────────┐
         │                       │
         │    EKF Filter Node    │
         │  (robot_localization) │
         │                       │
         └───────────┬───────────┘
                     │
                     │ /odometry/filtered
                     │ (50 Hz)
                     │
         ┌───────────┴───────────┐
         │                       │
         ▼                       ▼
┌─────────────────┐     ┌─────────────────┐
│   TF Broadcast  │     │  Navigation /   │
│  odom→base_link │     │     SLAM        │
└─────────────────┘     └─────────────────┘
```

## Frame Configuration

### TF Tree

```
map (from SLAM - Phase 4)
 │
 └── odom (from EKF)
      │
      └── base_footprint (from URDF)
           │
           └── base_link (from URDF)
                │
                ├── imu_link
                ├── camera_link
                └── ... 
```

### Important Notes

1. **EKF publishes odom → base_link TF** (not base_footprint)
2. **robot_state_publisher publishes base_footprint → base_link** (static, from URDF)
3. **SLAM will publish map → odom** (in Phase 4)

## Covariance Configuration

### Measurement Covariance

Each sensor publishes its own covariance, representing measurement uncertainty:

```
Wheel Odometry Covariance (from vex_serial_node):
  Position: 0.01 m²
  Yaw: 0.03 rad²
  Velocity: 0.01 (m/s)²

IMU Covariance (from imu_node):
  Orientation: 0.01 rad²
  Angular Velocity: 0.001 (rad/s)²
  Linear Acceleration: 0.01 (m/s²)²
```

### Process Noise

How much we expect the state to change between updates:

```
Position: 0.05 m²/update
Yaw: 0.06 rad²/update
Velocity: 0.025 (m/s)²/update
```

## Performance Characteristics

### Expected Accuracy

| Metric | Wheel Odom Only | EKF Fused |
|--------|----------------|-----------|
| Position drift (1m travel) | 5-10 cm | 2-5 cm |
| Heading drift (360° turn) | 5-10° | 1-3° |
| Recovery from slip | Poor | Good |

### Update Rates

- EKF internal rate: 50 Hz
- Output rate: 50 Hz
- Latency: < 20 ms

## Debugging

### Verify Fusion is Working

```bash
# Compare raw vs filtered
ros2 run tmr_localization odom_comparison.py

# View EKF diagnostics
ros2 run tmr_localization ekf_diagnostics.py
```

### Common Issues

1. **Filtered matches raw exactly**: IMU data not being received or not configured correctly
2. **Filtered very different from raw**: Covariance misconfiguration
3. **Position jumps**: Timestamp synchronization issues