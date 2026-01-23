# Nav2 Integration Guide

## Overview

This guide explains how to integrate the ToF Terrain Layer with Nav2 for the Terrain Mapping Rover.

## Prerequisites

1. Nav2 installed: `sudo apt install ros-jazzy-nav2-bringup`
2. TMR packages built and working
3. Phases 1-4 completed (hardware, TF, EKF, SLAM)

## Quick Start

```bash
# Start hardware + EKF (Phase 1-3)
ros2 launch tmr_bringup full_localization.launch.py

# Start SLAM (Phase 4)
ros2 launch tmr_slam slam.launch.py

# Start Navigation with Terrain Layer (Phase 5)
ros2 launch tmr_costmap navigation.launch.py
```

## Costmap Configuration

### Local Costmap

The local costmap is used for obstacle avoidance:

```yaml
local_costmap:
  plugins: ["tof_terrain_layer", "inflation_layer"]
  
  tof_terrain_layer:
    plugin: "tmr_costmap/ToFTerrainLayer"
    enabled: true
    point_cloud_topic: "/tof/points"
    observation_persistence: 1.0
    # ... terrain thresholds
  
  inflation_layer:
    plugin: "nav2_costmap_2d::InflationLayer"
    inflation_radius: 0.35
    cost_scaling_factor: 3.0
```

### Global Costmap

The global costmap includes SLAM map + terrain:

```yaml
global_costmap:
  plugins: ["static_layer", "tof_terrain_layer", "inflation_layer"]
  
  static_layer:
    plugin: "nav2_costmap_2d::StaticLayer"
    map_subscribe_transient_local: True
```

## Topic Connections

| Nav2 Topic | TMR Topic | Purpose |
|------------|-----------|---------|
| `/odom` | `/odometry/filtered` | Robot pose |
| `/map` | `/map` (from SLAM) | Static map |
| Point cloud | `/tof/points` | Terrain data |
| `/cmd_vel` | `/cmd_vel` | Velocity commands |

## Controller Configuration

### DWB Controller (Recommended)

```yaml
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"
  max_vel_x: 0.3       # Limit for terrain
  max_vel_theta: 0.5
  min_vel_x: 0.0
  acc_lim_x: 0.5
  acc_lim_theta: 1.0
```

## Testing Navigation

### 1. Verify Costmap

```bash
# Check costmap is updating
ros2 topic hz /local_costmap/costmap

# Visualize in RViz
ros2 launch tmr_costmap costmap_test.launch.py
```

### 2. Send Navigation Goals

```bash
# Use RViz "2D Goal Pose" tool
# Or command line:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### 3. Monitor Navigation

```bash
# Watch path planning
ros2 topic echo /plan

# Watch controller output
ros2 topic echo /cmd_vel
```

## Troubleshooting

### "No valid path found"

1. Check costmap shows free space
2. Verify robot footprint fits
3. Check inflation radius

### "Robot stops at obstacles"

1. Verify terrain thresholds are appropriate
2. Check costmap update rate
3. Ensure ToF data is current

### "Path goes through obstacles"

1. Check terrain layer is enabled
2. Verify global costmap includes terrain
3. Check layer order (terrain before inflation)

## Performance Tuning

### For Rough Terrain

```yaml
# Lower thresholds for caution
low_obstacle_height: 0.05
max_traversable_slope: 10.0

# Slower speeds
max_vel_x: 0.2
```

### For Speed

```yaml
# Higher thresholds
low_obstacle_height: 0.15
max_traversable_slope: 20.0

# Faster speeds
max_vel_x: 0.4
```

## Full System Launch

For complete autonomous navigation:

```bash
# Terminal 1: Hardware + Localization
ros2 launch tmr_bringup full_localization.launch.py

# Terminal 2: SLAM
ros2 launch tmr_slam mapping.launch.py

# Terminal 3: Navigation
ros2 launch tmr_costmap navigation.launch.py rviz:=true

# Terminal 4: Send goals via RViz or:
ros2 run nav2_simple_commander goal_pose_navigator
```