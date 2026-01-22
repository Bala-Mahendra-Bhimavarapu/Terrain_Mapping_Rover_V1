# Phase 3 Testing Procedure

## Overview

Phase 3 validates the EKF sensor fusion system that combines wheel odometry and IMU data into a more accurate pose estimate. 

## Prerequisites

### Hardware Checklist
- [ ] VEX V5 Brain connected and powered
- [ ] All motors connected
- [ ] MPU6050 IMU connected
- [ ] Robot on flat surface with space to move
- [ ] Emergency stop procedure known

### Software Checklist
- [ ] Phase 1 tests passing
- [ ] Phase 2 tests passing
- [ ] robot_localization package installed
- [ ] tmr_localization package built

## Test Sequence

### Step 1: Launch Phase 3 System

```bash
# Full Phase 3 bringup
ros2 launch tmr_bringup phase3_bringup.launch.py

# Or with teleop enabled
ros2 launch tmr_bringup phase3_bringup. launch.py teleop:=keyboard
```

**Expected**: All nodes start, no errors.

### Step 2: Verify Sensor Inputs

```bash
# Check wheel odometry
ros2 topic hz /vex/odom_raw
# Expected: ~50 Hz

# Check IMU
ros2 topic hz /imu/data
# Expected: ~100 Hz

# Check filtered output
ros2 topic hz /odometry/filtered
# Expected: ~50 Hz
```

### Step 3: Run EKF Check

```bash
ros2 run tmr_bringup phase3_ekf_check.py
```

**Expected Results**:
- ✅ Sensor inputs receiving data
- ✅ EKF producing output
- ✅ Output values valid
- ✅ Covariance reasonable
- ✅ Frame IDs correct

### Step 4: Run Fusion Test (Stationary)

```bash
ros2 run tmr_bringup phase3_fusion_test. py
```

**Robot must be STATIONARY for this test.**

**Expected Results**:
- ✅ IMU data contributing
- ✅ Yaw fusion working
- ✅ Position consistent
- ✅ Covariance stable

### Step 5: Run Odometry Test (With Motion)

⚠️ **WARNING:  Robot will move! **

```bash
ros2 run tmr_bringup phase3_odom_test.py
```

**Expected Results**:
- ✅ Forward motion tracked correctly
- ✅ Rotation tracked correctly
- ✅ Raw and filtered values reasonably close

### Step 6: Run Integration Test

⚠️ **WARNING: Robot will move! **

```bash
ros2 run tmr_bringup phase3_integration_test.py
```

**Expected Results**:
- ✅ All sensors running at expected rates
- ✅ TF publishing correctly
- ✅ Position updates with motion
- ✅ TF updates with motion
- ✅ IMU improves rotation estimate

### Step 7: Manual Verification

1. **Position Tracking Test**:
   ```bash
   # Terminal 1: Start system with teleop
   ros2 launch tmr_bringup phase3_bringup.launch.py teleop:=keyboard
   
   # Terminal 2: Monitor filtered odometry
   ros2 topic echo /odometry/filtered/pose/pose/position
   ```
   - Drive robot forward 1 meter
   - Check position shows ~1.0 in X
   - Return to start
   - Check position near (0, 0)

2. **Rotation Test**:
   - Rotate robot 360°
   - Check yaw returns to starting value
   - IMU should help reduce drift

3. **Square Test**:
   - Drive in a square pattern (1m x 1m)
   - Return to starting position
   - Position error should be < 10cm

## Troubleshooting

### EKF Not Producing Output

```bash
# Check EKF node is running
ros2 node list | grep ekf

# Check for errors
ros2 run robot_localization ekf_node --ros-args -p debug:=true
```

**Common Causes**:
- Wrong topic names in config
- TF tree incomplete
- Sensor data not arriving

### Position Jumps

**Symptoms**:  Position suddenly changes by large amount

**Solutions**:
1. Check sensor timestamps are synchronized
2. Increase sensor covariance
3. Reduce process noise

### Filtered Matches Raw Exactly

**Symptoms**: Filtered odometry identical to raw

**Cause**: IMU data not being used

**Solutions**:
1. Check IMU topic name matches config
2. Verify IMU covariance is not -1 (disabled)
3. Check `imu0_config` in EKF params

### Excessive Drift

**Symptoms**: Position drifts when stationary

**Solutions**:
1. Calibrate IMU
2. Reduce IMU acceleration trust
3. Check for vibration affecting IMU

## Comparing Raw vs Filtered

Use the comparison tool: 

```bash
ros2 run tmr_localization odom_comparison.py
```

**What to Look For**:
- Filtered should be smoother than raw
- During turns, filtered orientation should be more stable
- Position difference should stay small (< 20cm typically)

## Data Recording

Record data for offline analysis:

```bash
ros2 bag record \
  /vex/odom_raw \
  /imu/data \
  /odometry/filtered \
  /tf \
  -o phase3_test_$(date +%Y%m%d_%H%M%S)
```

## Success Criteria

| Metric | Requirement |
|--------|-------------|
| EKF output rate | ≥ 30 Hz |
| Position accuracy (1m travel) | < 10 cm error |
| Rotation accuracy (360°) | < 10° error |
| Square return error | < 15 cm |
| Stationary drift (1 min) | < 5 cm |

## Sign-Off

| Test | Status | Notes |
|------|--------|-------|
| EKF Check | ☐ Pass ☐ Fail | |
| Fusion Test | ☐ Pass ☐ Fail | |
| Odometry Test | ☐ Pass ☐ Fail | |
| Integration Test | ☐ Pass ☐ Fail | |
| Manual Verification | ☐ Pass ☐ Fail | |

**Tester:** _________________
**Date:** _________________
**Notes:**

## Next Steps

Once Phase 3 is complete: 
1. Save EKF configuration
2. Record baseline performance metrics
3. Proceed to Phase 4: SLAM