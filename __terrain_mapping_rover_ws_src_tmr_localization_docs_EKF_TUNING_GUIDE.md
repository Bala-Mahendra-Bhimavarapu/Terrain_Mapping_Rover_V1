# EKF Tuning Guide

## Overview

This guide explains how to tune the Extended Kalman Filter (EKF) for optimal sensor fusion on the Terrain Mapping Rover. 

## What the EKF Does

The EKF fuses two sensor inputs:
1. **Wheel Odometry** (from VEX serial node): Position (x, y), orientation (yaw), velocities
2. **IMU Data** (from MPU6050): Orientation (yaw), angular velocity, linear acceleration

The output is a more accurate estimate of the robot's pose than either sensor alone.

## Key Concepts

### Covariance

Covariance represents uncertainty in measurements: 
- **Low covariance** = High confidence in the measurement
- **High covariance** = Low confidence in the measurement

The EKF weighs measurements by their covariance - it trusts low-covariance measurements more. 

### Process Noise

Process noise represents how much the robot's state is expected to change between filter updates:
- **Low process noise** = Smooth output, slow response to changes
- **High process noise** = Responsive but noisier output

## Tuning Procedure

### Step 1: Verify Sensors Working

Before tuning, ensure both sensors are publishing data:

```bash
# Check wheel odometry
ros2 topic hz /vex/odom_raw

# Check IMU
ros2 topic hz /imu/data

# View data
ros2 topic echo /vex/odom_raw --once
ros2 topic echo /imu/data --once
```

### Step 2: Start with Default Parameters

Launch the EKF with default parameters: 

```bash
ros2 launch tmr_localization localization. launch.py
```

### Step 3: Evaluate Baseline Performance

1. Drive the robot in a straight line (1 meter)
2. Return to the starting point
3. Check the filtered odometry - it should show position near (0, 0)

```bash
ros2 topic echo /odometry/filtered/pose/pose/position
```

### Step 4: Tune Wheel Odometry Covariance

If the robot drifts when driving straight:

**Problem**: Position drifts laterally
**Solution**: Increase `odom_y` covariance (trust wheel odometry less for Y)

**Problem**:  Heading drifts
**Solution**:  Increase `odom_yaw` covariance (trust IMU yaw more)

Edit `config/ekf_params.yaml`:
```yaml
# In odom0_config, the covariance comes from the sensor
# To adjust, modify the sensor node's published covariance
# Or use differential mode
```

### Step 5: Tune IMU Covariance

If the robot oscillates or is too jittery:

**Problem**:  Orientation oscillates
**Solution**: Increase `imu_yaw` covariance in IMU config

**Problem**: Position jumps when accelerating
**Solution**: Increase `imu_ax` covariance or reduce trust in acceleration

### Step 6: Tune Process Noise

If the filter is too sluggish: 
- Increase process noise values

If the filter is too noisy:
- Decrease process noise values

```yaml
process_noise_covariance: 
  # Position (indices 0, 1, 2)
  [0, 0]:  0.05    # Increase for faster response
  
  # Orientation (index 5 for yaw in 2D)
  [5, 5]: 0.06    # Yaw process noise
  
  # Velocities (indices 6, 11)
  [6, 6]: 0.025   # Linear velocity
  [11, 11]: 0.02  # Angular velocity
```

## Common Scenarios

### Scenario 1: Robot on Carpet (Good Traction)

Trust wheel odometry more: 
```yaml
# Lower covariance = more trust
odom_x_covariance:  0.005
odom_y_covariance: 0.005
odom_yaw_covariance:  0.02
```

### Scenario 2: Robot on Slippery Surface

Trust IMU more:
```yaml
# Higher odom covariance = less trust
odom_x_covariance:  0.1
odom_y_covariance: 0.1
odom_yaw_covariance: 0.1

# Keep IMU covariance low
imu_yaw_covariance: 0.01
```

### Scenario 3: Rough Terrain (Vibration)

Trust IMU less for acceleration:
```yaml
# IMU may be noisy from vibration
imu_ax_covariance: 0.1
imu_ay_covariance: 0.1
```

## Using the Tuning Tools

### Interactive Tuner

```bash
ros2 launch tmr_localization ekf_tuning. launch.py
```

This launches: 
- EKF node
- Diagnostics display
- Interactive covariance tuner
- Plots for visualization

### Comparing Raw vs Filtered

```bash
ros2 run tmr_localization odom_comparison.py
```

Watch the error statistics to evaluate tuning changes.

## Troubleshooting

### EKF Not Producing Output

1. Check sensor data is being received
2. Verify TF tree is complete
3. Check for timestamp issues

```bash
# View EKF debug output
ros2 run robot_localization ekf_node --ros-args -p debug: =true
```

### Position Jumps

- Usually caused by large covariance updates
- Check for sensor dropouts
- Verify sensor timestamps are synchronized

### Slow Response

- Increase process noise
- Decrease sensor covariances
- Check filter frequency is high enough

## Best Practices

1. **Tune one parameter at a time**
2. **Test on flat surface first**, then add complexity
3. **Log data** for offline analysis
4. **Document your changes** and their effects
5. **Save working configurations** before experimenting

## Configuration Files

- `config/ekf_params.yaml` - Main EKF configuration
- `config/ekf_tuning.yaml` - Tuning presets and documentation
- `config/sensor_covariances.yaml` - Sensor noise parameters

## References

- [robot_localization Wiki](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
- [REP 105 - Coordinate Frames](https://www.ros.org/reps/rep-0105.html)
- [Kalman Filter Tutorial](https://www.kalmanfilter.net/default.aspx)