# Phase 1 Testing Procedure

## Overview

This document describes the complete testing procedure for Phase 1 of the Terrain Mapping Rover project.  Phase 1 focuses on validating all hardware connections, motor control, sensor data streams, and basic teleoperation. 

## Prerequisites

Before starting Phase 1 testing: 

### Hardware Checklist
- [ ] VEX V5 Brain powered on and connected via USB
- [ ] All 4 motors connected and secured
- [ ] MPU6050 IMU connected via I2C
- [ ] IMX500 camera connected via CSI
- [ ] Arducam ToF camera connected via CSI
- [ ] Robot on a flat surface with space to move
- [ ] Emergency stop procedure known

### Software Checklist
- [ ] ROS 2 Jazzy installed
- [ ] Workspace built:  `colcon build`
- [ ] Workspace sourced: `source install/setup.bash`
- [ ] All packages compiled without errors

## Test Sequence

### Step 1: System Check

**Purpose:** Verify all nodes can start and communicate. 

```bash
# Terminal 1: Start the full system
ros2 launch tmr_bringup phase1_bringup.launch.py

# Terminal 2: Run system check
ros2 run tmr_bringup phase1_system_check.py
```

**Expected Results:**
- All nodes running
- All topics publishing
- TF tree valid
- No error messages

**Pass Criteria:**
- ✅ All expected nodes listed
- ✅ All topics active
- ✅ No connection errors

### Step 2: Sensor Testing

**Purpose:** Verify all sensors produce valid data at expected rates.

```bash
# With system still running from Step 1
ros2 run tmr_bringup phase1_sensor_test.py
```

**Expected Results:**

| Sensor | Expected Rate | Minimum Rate |
|--------|--------------|--------------|
| IMU | 100 Hz | 80 Hz |
| Camera | 30 Hz | 10 Hz |
| ToF Camera | 15 Hz | 10 Hz |
| Odometry | 50 Hz | 30 Hz |

**Manual Sensor Verification:**

```bash
# Check IMU data
ros2 topic echo /imu/data --once

# Check camera image
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# Check ToF depth
ros2 run rqt_image_view rqt_image_view /tof/depth/image_colored

# Check odometry
ros2 topic echo /odom --once
```

### Step 3: Motor Testing

**Purpose:** Verify motor control and encoder feedback. 

⚠️ **WARNING:** Robot will move during this test! 

```bash
ros2 run tmr_bringup phase1_motor_test. py
```

**Tests Performed:**
1. Forward motion (2 seconds)
2. Backward motion (2 seconds)
3. Turn left (2 seconds)
4. Turn right (2 seconds)
5. Emergency stop

**Pass Criteria:**
- ✅ Robot moves in correct direction
- ✅ Distance traveled matches commanded velocity
- ✅ Encoder readings are consistent
- ✅ Emergency stop works immediately

### Step 4: Teleop Testing

**Purpose:** Verify manual control works correctly.

```bash
# Start teleop mode
ros2 launch tmr_bringup phase1_teleop.launch.py

# For gamepad instead of keyboard: 
ros2 launch tmr_bringup phase1_teleop. launch.py mode: =gamepad
```

**Manual Tests:**

| Test | Action | Expected |
|------|--------|----------|
| Forward | Press W | Robot moves forward |
| Backward | Press S | Robot moves backward |
| Turn Left | Press A | Robot turns left |
| Turn Right | Press D | Robot turns right |
| Stop | Press X | Robot stops gradually |
| E-Stop | Press SPACE | Robot stops immediately |
| Speed Up | Press I | Linear speed increases |
| Speed Down | Press K | Linear speed decreases |

### Step 5: Integration Testing

**Purpose:** Verify all components work together.

```bash
ros2 run tmr_bringup phase1_integration_test. py
```

**Tests Performed:**
1. Teleop → Motor response
2. Motor motion → Odometry update
3. Odometry → TF broadcast
4. All sensors during motion
5. IMU motion detection
6. Emergency stop integration

### Step 6: Calibration (Optional)

**Purpose:** Fine-tune sensor parameters.

```bash
ros2 run tmr_bringup phase1_calibration.py
```

**Calibrations:**
1. IMU bias calibration
2. Odometry distance calibration
3. Odometry rotation calibration

## Troubleshooting

### VEX Not Connecting

```bash
# Check USB connection
ls /dev/ttyACM*

# Check permissions
sudo chmod 666 /dev/ttyACM0

# Try different port
ros2 run tmr_vex_serial vex_serial_node --ros-args -p serial_port:=/dev/ttyACM1
```

### IMU Not Detected

```bash
# Check I2C connection
i2cdetect -y 1

# Should show device at 0x68
# If not, check wiring
```

### Camera Not Working

```bash
# Check camera detection
libcamera-hello --list-cameras

# Check if camera module enabled
sudo raspi-config
# Interface Options -> Camera -> Enable
```

### Motors Not Moving

1. Check VEX Brain is in "Driver Control" mode
2. Verify motor port assignments match config
3. Check for motor faults on VEX Brain LCD
4. Verify motor cables are secure

## Data Recording

To record test data for later analysis:

```bash
# Record all Phase 1 topics
ros2 bag record \
  /cmd_vel \
  /odom \
  /imu/data \
  /camera/image_raw/compressed \
  /tof/depth/image_raw \
  /tf \
  -o phase1_test_$(date +%Y%m%d_%H%M%S)
```

## Sign-Off

After completing all tests:

| Test | Status | Notes |
|------|--------|-------|
| System Check | ☐ Pass ☐ Fail | |
| Sensor Test | ☐ Pass ☐ Fail | |
| Motor Test | ☐ Pass ☐ Fail | |
| Teleop Test | ☐ Pass ☐ Fail | |
| Integration Test | ☐ Pass ☐ Fail | |

**Tester:** _________________
**Date:** _________________
**Notes:** _________________

## Next Steps

Once all Phase 1 tests pass: 

1. Save calibration data
2. Record baseline performance metrics
3. Proceed to Phase 2: Sensor Fusion & SLAM