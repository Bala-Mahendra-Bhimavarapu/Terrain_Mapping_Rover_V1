# Phase 2 Testing Procedure

## Overview

Phase 2 validates the TF (Transform) tree setup for the Terrain Mapping Rover. A correct TF tree is essential for sensor fusion and SLAM in later phases. 

## Prerequisites

### Hardware Checklist
- [ ] No hardware required for Phase 2 TF testing
- [ ] Robot can be powered off

### Software Checklist
- [ ] Workspace compiled:  `colcon build`
- [ ] Workspace sourced: `source install/setup.bash`
- [ ] tmr_description package built successfully

## Test Sequence

### Step 1: Launch Phase 2 Bringup

```bash
# Start Phase 2 with static transforms for testing
ros2 launch tmr_bringup phase2_bringup.launch. py

# With RViz visualization
ros2 launch tmr_bringup phase2_bringup.launch.py rviz:=true
```

**Expected**:  No errors, all nodes start successfully. 

### Step 2: Verify TF Tree Exists

```bash
# In another terminal
# Check TF tree structure
ros2 run tf2_tools view_frames

# View the generated PDF
evince frames.pdf
```

**Expected**: PDF shows complete tree from `map` down to all sensor frames.

### Step 3: Run TF Check Script

```bash
ros2 run tmr_bringup phase2_tf_check.py
```

**Expected Results**:
- ✅ All transforms exist
- ✅ No NaN values
- ✅ Quaternions normalized

### Step 4: Run Transform Value Test

```bash
ros2 run tmr_bringup phase2_transform_test.py
```

**Expected Results**:
| Transform | Expected Position |
|-----------|------------------|
| base_footprint → base_link | (0, 0, 0.0508) |
| base_link → imu_link | (0, 0, 0.046) |
| base_link → camera_link | (0.13, -0.0275, 0.078) |
| base_link → tof_link | (0.13, 0.014, 0.078) |

### Step 5: Run Full TF Test Suite

```bash
ros2 launch tmr_bringup phase2_tf_test.launch.py test: =all
```

**Expected**: All tests pass. 

### Step 6: Verify in RViz

1. Launch with RViz:  `ros2 launch tmr_bringup phase2_bringup.launch.py rviz:=true`
2. Add TF display
3. Add RobotModel display
4. Verify: 
   - [ ] Robot model visible
   - [ ] All TF frames shown
   - [ ] Frames in correct positions

## Troubleshooting

### "Transform not available"

```bash
# Check which transforms exist
ros2 run tf2_ros tf2_echo base_link camera_link

# List all frames
ros2 run tf2_ros tf2_monitor
```

**Solutions**:
- Verify robot_state_publisher is running
- Check URDF is valid:  `check_urdf $(ros2 pkg prefix tmr_description)/share/tmr_description/urdf/rover.urdf. xacro`

### "Transform has NaN"

**Solution**:  Check URDF joint origins for invalid values.

### "Quaternion not normalized"

**Solution**: Check rotation values in URDF are valid.

## Success Criteria

- [ ] All expected frames exist
- [ ] All transforms have valid values
- [ ] RViz shows correct robot model
- [ ] No errors in console

## Sign-Off

| Test | Status | Notes |
|------|--------|-------|
| TF Tree Generation | ☐ Pass ☐ Fail | |
| TF Check Script | ☐ Pass ☐ Fail | |
| Transform Values | ☐ Pass ☐ Fail | |
| RViz Visualization | ☐ Pass ☐ Fail | |

**Tester:** _________________
**Date:** _________________
