# TF Tree Documentation

## Overview

This document describes the TF (Transform) tree structure for the Terrain Mapping Rover. 

## TF Tree Structure

```
map                          [Published by:  SLAM (Phase 3)]
 │
 └── odom                    [Published by: EKF or vex_serial_node]
      │
      └── base_footprint     [Static:  robot_state_publisher]
           │
           └── base_link     [Static: robot_state_publisher]
                │
                ├── front_left_wheel      [Static]
                ├── front_right_wheel     [Static]
                ├── back_left_wheel       [Static]
                ├── back_right_wheel      [Static]
                │
                ├── imu_link              [Static]
                │
                ├── camera_link           [Static]
                │    └── camera_optical_frame  [Static]
                │
                └── tof_link              [Static]
                     └── tof_optical_frame     [Static]
```

## Frame Descriptions

### World Frames

| Frame | Description | Publisher |
|-------|-------------|-----------|
| `map` | Global fixed frame, origin at map start | SLAM (slam_toolbox) |
| `odom` | Odometry frame, drifts over time | EKF or vex_serial_node |

### Robot Frames

| Frame | Description | Parent |
|-------|-------------|--------|
| `base_footprint` | On ground plane, directly below robot | `odom` |
| `base_link` | Center of robot, at wheel axle height | `base_footprint` |

### Sensor Frames

| Frame | Description | Parent | Position (from base_link) |
|-------|-------------|--------|---------------------------|
| `imu_link` | MPU6050 IMU location | `base_link` | (0, 0, 0.046) m |
| `camera_link` | IMX500 camera body | `base_link` | (0.13, -0.0275, 0.078) m |
| `camera_optical_frame` | Camera optical center | `camera_link` | Z forward, X right, Y down |
| `tof_link` | Arducam ToF body | `base_link` | (0.13, 0.014, 0.078) m |
| `tof_optical_frame` | ToF optical center | `tof_link` | Z forward, X right, Y down |

### Wheel Frames

| Frame | Description | Position (from base_link) |
|-------|-------------|---------------------------|
| `front_left_wheel` | Front left wheel center | (0.0835, 0.1475, 0) m |
| `front_right_wheel` | Front right wheel center | (0.0835, -0.1475, 0) m |
| `back_left_wheel` | Back left wheel center | (-0.0835, 0.1475, 0) m |
| `back_right_wheel` | Back right wheel center | (-0.0835, -0.1475, 0) m |

## Coordinate Conventions

### ROS Standard (REP 103)

- **X**:  Forward (positive)
- **Y**: Left (positive)
- **Z**: Up (positive)

### Optical Frame Convention (REP 103)

For cameras and depth sensors:
- **Z**: Forward (into the scene)
- **X**: Right
- **Y**: Down

The transform from `camera_link` to `camera_optical_frame` applies:
- -90° rotation about Z
- -90° rotation about X

## Transform Publishers

### Static Transforms (from URDF)

Published by `robot_state_publisher` at startup:
- `base_footprint` → `base_link`
- `base_link` → all wheel frames
- `base_link` → `imu_link`
- `base_link` → `camera_link`
- `camera_link` → `camera_optical_frame`
- `base_link` → `tof_link`
- `tof_link` → `tof_optical_frame`

### Dynamic Transforms

| Transform | Publisher | Rate | Notes |
|-----------|-----------|------|-------|
| `odom` → `base_footprint` | `vex_serial_node` | 50 Hz | Wheel odometry |
| `odom` → `base_link` | `robot_localization` (EKF) | 50 Hz | Fused odometry (Phase 2+) |
| `map` → `odom` | `slam_toolbox` | 10 Hz | SLAM correction (Phase 3) |

## Debugging TF

### View TF Tree

```bash
# Generate PDF of TF tree
ros2 run tf2_tools view_frames

# View the PDF
evince frames.pdf
```

### Echo Specific Transform

```bash
# Echo transform between two frames
ros2 run tf2_ros tf2_echo base_link camera_optical_frame

# Monitor transform rates
ros2 run tf2_ros tf2_monitor
```

### Check for Issues

```bash
# List all frames
ros2 run tf2_ros tf2_monitor

# Run TF validator
ros2 run tmr_description tf_validator. py
```

## Common Issues

### "Could not find transform"

**Cause**: Transform not being published or tree is broken. 

**Solution**:
1. Check that `robot_state_publisher` is running
2. Verify URDF is valid:  `check_urdf rover.urdf`
3. Check TF tree:  `ros2 run tf2_tools view_frames`

### "Transform timeout"

**Cause**: Transform is stale or published too slowly.

**Solution**:
1. Check publisher node is running
2. Verify publish rate is sufficient
3. Increase TF buffer timeout if needed

### "Extrapolation into the future"

**Cause**: Timestamps are out of sync (clock issues).

**Solution**:
1. Ensure all nodes use same clock source
2. Check `use_sim_time` parameter consistency
3. Verify system clock is synchronized

## Measurements Reference

All measurements in meters from `base_link` center: 

```
                    FRONT
                      │
        ┌─────────────┼─────────────┐
        │   FL        │        FR   │
        │   ●─────────┼──���──────●   │
        │             │             │
        │      ┌──────┼──────┐      │
        │      │  IMU │      │      │
        │      │   ●  │      │      │
        │      │      ● base_link   │
        │      │      │      │      │
        │      └──────┼──────┘      │
        │             │             │
        │   ●─────────┼─────────●   │
        │   BL        │        BR   │
        └─────────────┼─────────────┘
                      │
                    BACK

Camera:  13cm forward, 2.75cm right, 7.8cm up
ToF:     13cm forward, 1.4cm left, 7.8cm up
IMU:    Centered, 4.6cm up
```