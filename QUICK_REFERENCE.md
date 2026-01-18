# 🚀 Lunar Rover Quick Reference Card

## Essential Commands

### Start the System

```bash
# Source workspace
source ~/terrain_mapping_rover_ws/install/setup.bash

# Manual driving (sensors + teleop)
ros2 launch rover_bringup manual_drive.launch.py

# Full autonomous system
ros2 launch rover_bringup full_system.launch.py

# With mock sensors (no hardware)
ros2 launch rover_bringup phase1_sensors.launch.py use_mock: =true
```

### Individual Components

```bash
# VEX Serial only
ros2 run vex_serial vex_serial_node. py

# IMU only
ros2 run imu_driver mpu6050_node.py

# Camera only
ros2 run imx500_camera imx500_camera_node.py

# ToF only
ros2 run arducam_tof arducam_tof_node.py

# Teleop
ros2 run rover_teleop keyboard_teleop_node.py

# Web dashboard
ros2 run web_dashboard dashboard_node.py
```

### Waypoint Recording

```bash
# Start recording
ros2 service call /waypoint_recorder/start std_srvs/srv/SetBool "{data:  true}"

# Stop recording
ros2 service call /waypoint_recorder/stop std_srvs/srv/Trigger

# Save waypoints
ros2 service call /waypoint_recorder/save std_srvs/srv/Trigger

# Follow waypoints
ros2 service call /waypoint_follower/start std_srvs/srv/Trigger
```

### Debugging

```bash
# List all topics
ros2 topic list

# Check topic rate
ros2 topic hz /vex/odom_raw

# Echo topic data
ros2 topic echo /odom

# View TF tree
ros2 run tf2_tools view_frames

# Check transforms
ros2 run tf2_ros tf2_echo odom base_link

# Launch RViz
rviz2 -d ~/terrain_mapping_rover_ws/src/rover_bringup/rviz/rover_default.rviz
```

### Calibration

```bash
# Camera calibration
ros2 run calibration_tools camera_calibration_node.py

# IMU calibration
ros2 run calibration_tools imu_calibration_node.py

# Odometry calibration
ros2 run calibration_tools odometry_calibration_node.py
```

## Key Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/vex/odom_raw` | Odometry | 50Hz | Raw wheel odometry |
| `/imu/data` | Imu | 100Hz | IMU data |
| `/odom` | Odometry | 50Hz | Fused odometry |
| `/camera/image_raw` | Image | 30Hz | RGB camera |
| `/tof/depth/image_raw` | Image | 15Hz | Depth image |
| `/tof/points` | PointCloud2 | 15Hz | 3D points |
| `/vex/cmd_vel` | Twist | - | Velocity commands |
| `/mission_health` | MissionHealth | 2Hz | System health |
| `/map` | OccupancyGrid | 1Hz | SLAM map |

## Key Parameters

### Robot Geometry (MUST match physical robot)
```yaml
wheel_radius: 0.0508      # 5.08 cm
track_width: 0.2921       # 29.21 cm
ticks_per_revolution: 900 # VEX V5 18: 1
```

### Motor Direction (for mirrored right motors)
```yaml
invert_left_encoder: false
invert_right_encoder:  true
```

### Velocity Limits
```yaml
max_linear_vel: 0.3   # m/s
max_angular_vel: 1.5  # rad/s
```

## Web Interfaces

| Service | URL | Description |
|---------|-----|-------------|
| Dashboard | http://pi:5000 | System monitoring |
| Camera Stream | http://pi:8080/stream | MJPEG stream |
| RViz | Local | 3D visualization |

## File Locations

```
~/terrain_mapping_rover_ws/     # ROS workspace
├── src/                        # Source packages
├── install/                    # Built packages
└── log/                        # Logs

~/.ros/
├── waypoints/                  # Saved waypoints
├── landmark_maps/              # Landmark maps
├── calibration/                # Calibration files
└── rtabmap. db                  # SLAM database

~/vex_v5_code/                  # VEX PROS project
```

## Teleop Controls

```
┌─────────────────────────────┐
│         W / ↑               │
│         Forward             │
│                             │
│   A / ←   SPACE   D / →     │
│   Left    STOP    Right     │
│                             │
│         S / ↓               │
│         Back                │
│                             │
│   +/-  Speed adjust         │
│   Shift  Boost              │
└─────────────────────────────┘
```

## Troubleshooting Quick Fixes

**Robot not moving:**
1. Check VEX connection:  `ros2 topic echo /vex/status`
2. Check cmd_vel:  `ros2 topic echo /vex/cmd_vel`
3. Verify V5 Brain is running program

**Odometry drifting:**
1. Calibrate wheel radius
2. Check encoder direction inversion
3. Tune EKF covariances

**No camera image:**
1. Check connection:  `vcgencmd get_camera`
2. Check topic: `ros2 topic hz /camera/image_raw`

**SLAM not working:**
1. Check image quality in RViz
2. Increase feature count in config
3. Check camera exposure
