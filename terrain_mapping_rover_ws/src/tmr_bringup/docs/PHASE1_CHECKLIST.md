# Phase 1 Completion Checklist

## Hardware Setup
- [ ] VEX V5 Brain connected and powered
- [ ] All 4 motors connected (ports 1-4)
- [ ] MPU6050 IMU wired to I2C
- [ ] IMX500 camera connected to CSI
- [ ] Arducam ToF connected to CSI
- [ ] All cables secured

## Software Build
- [ ] Workspace compiles without errors
- [ ] All packages built successfully
- [ ] No missing dependencies

## System Check
- [ ] All nodes start successfully
- [ ] All topics publishing
- [ ] All services available
- [ ] TF tree complete

## Sensor Validation
- [ ] IMU producing data at ≥80 Hz
- [ ] IMU accelerometer reads ~9.81 m/s² (gravity)
- [ ] IMU gyroscope stable when stationary
- [ ] Camera producing images at ≥10 fps
- [ ] Camera images properly exposed
- [ ] ToF producing depth at ≥10 fps
- [ ] ToF point cloud generated

## Motor Control
- [ ] Forward motion works
- [ ] Backward motion works
- [ ] Left turn works
- [ ] Right turn works
- [ ] Motors respond to cmd_vel
- [ ] Emergency stop works

## Odometry
- [ ] Odometry updates with motion
- [ ] Position tracking reasonable
- [ ] TF odom→base_link published

## Teleop
- [ ] Keyboard control works
- [ ] Gamepad control works (if applicable)
- [ ] Speed adjustment works
- [ ] E-stop from teleop works

## Calibration
- [ ] IMU calibrated
- [ ] Odometry distance calibrated
- [ ] Odometry rotation calibrated

## Integration
- [ ] All sensors work during motion
- [ ] No data dropouts
- [ ] System stable for 10+ minutes

---

**Completed By:** _________________
**Date:** _________________
**Notes:** 
