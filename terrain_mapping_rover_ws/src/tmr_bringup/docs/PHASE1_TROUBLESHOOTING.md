# Phase 1 Troubleshooting Guide

## Common Issues and Solutions

### Connection Issues

#### VEX V5 Brain Not Detected

**Symptoms:**
- `/vex/connected` shows `false`
- Serial port errors in logs

**Solutions:**

1. **Check USB Cable**
   ```bash
   # List USB devices
   lsusb
   
   # Check serial ports
   ls -la /dev/ttyACM*
   ```

2. **Fix Permissions**
   ```bash
   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   
   # Or set permissions directly
   sudo chmod 666 /dev/ttyACM0
   
   # Create udev rule for permanent fix
   echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="2888", MODE="0666"' | \
     sudo tee /etc/udev/rules. d/99-vex. rules
   sudo udevadm control --reload-rules
   ```

3. **Check VEX Brain Status**
   - Ensure VEX Brain is powered on
   - Check that correct program is running
   - Verify "Driver Control" mode is active

4. **Try Different Port**
   ```bash
   ros2 run tmr_vex_serial vex_serial_node --ros-args \
     -p serial_port: =/dev/ttyACM1
   ```

#### IMU (MPU6050) Not Detected

**Symptoms:**
- `/imu/connected` shows `false`
- I2C errors in logs

**Solutions:**

1. **Check I2C Bus**
   ```bash
   # Enable I2C if not already
   sudo raspi-config
   # Interface Options -> I2C -> Enable
   
   # Scan I2C bus
   i2cdetect -y 1
   
   # Should show 68 (or 69) in the grid
   ```

2. **Check Wiring**
   ```
   MPU6050     Raspberry Pi 5
   --------    --------------
   VCC    -->  3.3V (Pin 1)
   GND    -->  GND (Pin 6)
   SDA    -->  GPIO2/SDA (Pin 3)
   SCL    -->  GPIO3/SCL (Pin 5)
   ```

3. **Check Address**
   ```bash
   # If AD0 pin is HIGH, address is 0x69
   ros2 run tmr_imu_driver imu_node --ros-args \
     -p i2c_address:=0x69
   ```

4. **Check I2C Speed**
   ```bash
   # Edit config. txt
   sudo nano /boot/firmware/config.txt
   
   # Add or modify:
   dtparam=i2c_arm=on
   dtparam=i2c_arm_baudrate=100000
   ```

#### Camera Not Working

**Symptoms:**
- `/camera/connected` shows `false`
- No image on `/camera/image_raw`

**Solutions:**

1. **Check Camera Detection**
   ```bash
   # List cameras
   libcamera-hello --list-cameras
   
   # Test camera
   libcamera-hello -t 5000
   ```

2. **Enable Camera Interface**
   ```bash
   sudo raspi-config
   # Interface Options -> Camera -> Enable
   sudo reboot
   ```

3. **Check CSI Connection**
   - Ensure ribbon cable is fully seated
   - Check cable orientation (blue side faces Ethernet port)
   - Try reseating the cable

4. **Check for Conflicts**
   ```bash
   # Kill any processes using camera
   sudo fuser -k /dev/video0
   ```

#### ToF Camera Not Working

**Symptoms:**
- `/tof/connected` shows `false`
- No depth data

**Solutions:**

1. **Install Arducam SDK**
   ```bash
   # Clone and install
   git clone https://github.com/ArduCAM/Arducam_tof_camera.git
   cd Arducam_tof_camera
   ./Install_dependencies.sh
   ./compile.sh
   ```

2. **Check Device Tree Overlay**
   ```bash
   # Edit config.txt
   sudo nano /boot/firmware/config.txt
   
   # Add: 
   dtoverlay=arducam-pivariety
   
   sudo reboot
   ```

3. **Test ToF Directly**
   ```bash
   cd Arducam_tof_camera/example
   ./preview_depth
   ```

---

### Motor Issues

#### Motors Not Moving

**Symptoms:**
- Commands sent but no motion
- Encoders not updating

**Solutions:**

1. **Check VEX Brain Mode**
   - Must be in "Driver Control" mode
   - Check for motor fault indicators on LCD

2. **Verify Motor Ports**
   ```yaml
   # In config file, ensure ports match physical connections: 
   motors:
     left_front_port: 1
     right_front_port: 2
     left_rear_port: 3
     right_rear_port: 4
   ```

3. **Check Motor Reversal**
   ```yaml
   # If robot moves opposite direction: 
   motors:
     reverse_left:  true
     reverse_right: false
   ```

4. **Test Individual Motors**
   ```bash
   # Send direct velocity command
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
   ```

#### Encoders Not Working

**Symptoms:**
- `/odom` position doesn't change
- `/wheel_encoders` shows zeros

**Solutions:**

1. **Check Encoder Connection**
   - VEX Smart Motors have built-in encoders
   - Ensure motor cables are fully connected

2. **Verify Encoder Parameters**
   ```yaml
   motors:
     ticks_per_revolution: 1800  # For 36: 1 cartridge
   ```

3. **Monitor Raw Encoder Data**
   ```bash
   ros2 topic echo /wheel_encoders
   ```

#### Robot Drives in Wrong Direction

**Solutions:**

1. **Swap Motor Reversal**
   ```yaml
   # Try inverting these values: 
   reverse_left_front: true   # was false
   reverse_right_front: false # was true
   ```

2. **Check Wheel Mounting**
   - Ensure wheels are on correct sides
   - Check that motors are mounted consistently

---

### Sensor Data Issues

#### IMU Data Invalid

**Symptoms:**
- Accelerometer reads 0 or very large values
- Gyroscope drifting excessively

**Solutions:**

1. **Recalibrate IMU**
   ```bash
   # Robot must be stationary and level! 
   ros2 service call /imu/calibrate std_srvs/srv/Trigger
   ```

2. **Check Mounting**
   - IMU should be mounted securely
   - Avoid mounting near motors (vibration)
   - Ensure correct orientation

3. **Verify Gravity Vector**
   ```bash
   # When level, Z acceleration should be ~9.81 m/s²
   ros2 topic echo /imu/data --field linear_acceleration
   ```

#### Camera Image Quality Issues

**Symptoms:**
- Dark or overexposed images
- Blurry images
- Low frame rate

**Solutions:**

1. **Adjust Exposure**
   ```bash
   ros2 run tmr_camera camera_node --ros-args \
     -p exposure_mode:=auto \
     -p exposure_compensation:=0.5
   ```

2. **Reduce Resolution for Higher FPS**
   ```bash
   ros2 run tmr_camera camera_node --ros-args \
     -p width:=640 -p height:=480 -p frame_rate:=30
   ```

3. **Check Lighting Conditions**
   - Ensure adequate lighting
   - Avoid direct sunlight on lens

#### ToF Depth Data Invalid

**Symptoms:**
- Large areas of zero depth
- Noisy depth readings
- Low valid point ratio

**Solutions:**

1. **Change Depth Mode**
   ```bash
   # For close objects (< 1.2m):
   ros2 service call /tof/set_near_mode std_srvs/srv/Trigger
   
   # For far objects (1-4m):
   ros2 service call /tof/set_far_mode std_srvs/srv/Trigger
   ```

2. **Adjust Confidence Threshold**
   ```bash
   ros2 run tmr_tof_camera tof_camera_node --ros-args \
     -p confidence_threshold:=20
   ```

3. **Check for Interference**
   - Avoid direct sunlight
   - Keep lens clean
   - Avoid highly reflective surfaces

---

### Odometry Issues

#### Odometry Drifting

**Symptoms:**
- Robot position drifts when stationary
- Accumulated error over time

**Solutions:**

1. **Calibrate Wheel Radius**
   ```bash
   ros2 run tmr_bringup phase1_calibration. py
   ```

2. **Calibrate Track Width**
   - Measure actual track width precisely
   - Update configuration

3. **Check for Wheel Slip**
   - Ensure good tire traction
   - Avoid smooth/slippery surfaces

#### Odometry Jumps

**Symptoms:**
- Sudden large changes in position
- Erratic position updates

**Solutions:**

1. **Check Encoder Connections**
   - Loose connections cause missed counts
   - Verify cable integrity

2. **Check for Overflow**
   - Encoder counters may overflow
   - Verify data types in VEX code

3. **Reduce Command Rate**
   ```yaml
   vex_serial_node:
     ros__parameters:
       encoder_publish_rate_hz: 50.0
   ```

---

### TF Issues

#### TF Tree Incomplete

**Symptoms:**
- Missing transforms
- "Could not find transform" errors

**Solutions:**

1. **Check Robot State Publisher**
   ```bash
   ros2 run tf2_ros tf2_echo base_link odom
   ```

2. **Visualize TF Tree**
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```

3. **Check URDF**
   ```bash
   ros2 param get /robot_state_publisher robot_description
   ```

#### TF Timestamp Issues

**Symptoms:**
- "Lookup would require extrapolation into the future"
- Transform warnings in logs

**Solutions:**

1. **Check System Clock**
   ```bash
   # Sync time
   sudo timedatectl set-ntp true
   ```

2. **Increase TF Buffer Duration**
   ```python
   tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
   ```

---

### Performance Issues

#### Low Frame Rates

**Solutions:**

1. **Reduce Resolution**
   ```yaml
   camera_node:
     ros__parameters:
       width: 640
       height: 480
   ```

2. **Disable Unused Features**
   ```yaml
   tof_camera_node:
     ros__parameters: 
       publish_pointcloud: false
       publish_amplitude_image: false
   ```

3. **Check CPU Usage**
   ```bash
   htop
   ```

#### High Latency

**Solutions:**

1. **Use Best-Effort QoS**
   ```python
   qos = QoSProfile(
       reliability=ReliabilityPolicy.BEST_EFFORT,
       depth=1
   )
   ```

2. **Reduce Queue Sizes**
   ```yaml
   queue_size:  1
   ```

3. **Check Network (if using remote PC)**
   ```bash
   ping <robot_ip>
   ```

---

## Diagnostic Commands

### Quick Status Check

```bash
# Check all nodes
ros2 node list

# Check all topics
ros2 topic list

# Check topic rates
ros2 topic hz /imu/data
ros2 topic hz /odom
ros2 topic hz /camera/image_raw

# Check TF
ros2 run tf2_ros tf2_monitor
```

### Log Analysis

```bash
# View logs
ros2 run rqt_console rqt_console

# Set log level
ros2 service call /imu_node/set_logger_level \
  rcl_interfaces/srv/SetLoggerLevel \
  "{logger_name: 'imu_node', level: 10}"
```

### System Diagnostics

```bash
# Check diagnostics
ros2 topic echo /diagnostics

# View in rqt
ros2 run rqt_robot_monitor rqt_robot_monitor
```

---

## Getting Help

If issues persist:

1. **Check Logs**
   ```bash
   ros2 launch tmr_bringup phase1_bringup.launch.py 2>&1 | tee launch. log
   ```

2. **Record Data for Analysis**
   ```bash
   ros2 bag record -a -o debug_bag
   ```

3. **Create Issue Report**
   - System information
   - Complete error messages
   - Steps to reproduce
   - Relevant log files
