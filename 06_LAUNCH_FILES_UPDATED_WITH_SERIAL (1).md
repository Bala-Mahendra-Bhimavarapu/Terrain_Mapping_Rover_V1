# 06_LAUNCH_FILES_UPDATED_WITH_SERIAL.md
## Complete Phase-by-Phase Launch Guide with Serial Communication
## Updated January 7, 2026 - Includes USB Serial Bridge Setup

**Status:** Complete 12-phase launch guide with serial communication  
**Total Phases:** 12 (Phase 0-11)  
**Serial Communication:** USB 115200 baud, 100 Hz updates  
**First Launch:** Phase 1 (includes serial bridge to V5 Brain)  

---

## 🎯 OVERVIEW

This guide covers **12 sequential phases:**

### **Phase 0:** Hardware Check (verification only)
### **Phase 1:** Sensor Bring-Up (INCLUDES SERIAL BRIDGE TO V5 BRAIN) ⭐
### **Phase 2:** Static TF Tree
### **Phase 3:** EKF Odometry Fusion
### **Phase 4:** SLAM (RTAB-Map)
### **Phase 5:** Terrain Layer
### **Phase 6:** Auto-Exposure
### **Phase 7:** Perception (Classifier + Detector)
### **Phase 8:** Localization & Health Monitor
### **Phase 9:** Nav2 (Static Map)
### **Phase 10:** Waypoint Recording/Playback
### **Phase 11:** Full Mission (30 min autonomous)

**Key:** Phase 1 includes the **serial bridge** that connects to V5 Brain via USB

---

## 🔌 PHASE 0: Hardware Check (Verification Only)

### Purpose
Verify all hardware is correctly connected before launching software

### Pre-Launch Verification

```bash
# 1. Check V5 Brain USB connection
ls -la /dev/ttyACM0
# Should show: crw-rw-rw- (or similar, readable)

# 2. Test V5 Brain console
pros terminal
# Should show startup messages
# Exit with: Ctrl+C

# 3. Check VEX motors are connected
# - V5 Brain screen should show motors on ports 1-4
# - Manual spin should show encoder values changing

# 4. Check camera connections
ls /dev/video*
# Should show: /dev/video0 (or similar)

# 5. Check IMU connection
i2cdetect -y 1
# Should show device at address 0x68 (MPU6050)

# 6. Check ToF camera
# Manual inspection of physical connection

# 7. Verify all cables
# - USB cable from V5 to Linux PC
# - I2C ribbon from IMU to Pi
# - Camera ribbon from IMX500 to Pi
# - Power connections to all devices

echo "✅ Hardware verification complete - ready for Phase 1"
```

### Troubleshooting

```bash
# If /dev/ttyACM0 not found:
# Solution 1: Restart V5 Brain
# Solution 2: Reconnect USB cable
# Solution 3: Check with: lsusb | grep VEX

# If V5 Brain console shows errors:
# Check PROS code uploaded correctly: pros upload
# Monitor V5 Brain startup: pros terminal

# If motors not showing on V5:
# Physically verify motors connected to ports 1-4
# Check V5 Brain port lights (green = connected)
```

---

## ⭐ PHASE 1: Sensor Bring-Up (WITH SERIAL BRIDGE TO V5 BRAIN)

### Purpose
Launch all 4 sensors and verify V5 Brain encoder data flowing via USB serial

### Prerequisites
✅ V5 Brain powered on
✅ USB cable connected to Linux PC
✅ All sensors physically connected
✅ Workspace sourced: `source ~/ros2_moon_rover/install/setup.bash`

### Launch Phase 1 (WITH Serial Bridge)

```bash
# 1. In Terminal 1: Start ROS 2 serial bridge
# This connects to V5 Brain on /dev/ttyACM0
source ~/ros2_moon_rover/install/setup.bash

# IMPORTANT: Start the serial bridge FIRST
ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

# Output should show:
# [INFO] ROS Serial Python Node
# [INFO] Connecting to /dev/ttyACM0 at 115200 baud

# Wait 3-5 seconds until it shows:
# [INFO] ROS Serial Client initialized

# ⚠️ DO NOT CLOSE THIS TERMINAL - Keep it running!
```

```bash
# 2. In Terminal 2: Launch Phase 1 sensors
source ~/ros2_moon_rover/install/setup.bash

ros2 launch rover_launch phase_1_sensors.launch.py

# Output should show:
# [INFO] Launching sensors...
# [INFO] VEX driver node started
# [INFO] IMU driver node started
# [INFO] Camera driver node started
# [INFO] ToF driver node started

# This will start:
# - vex_driver_node (subscribes to /vex/odom_raw from serial bridge)
# - imu_driver_node (reads /imu/data)
# - imx500_camera_node (reads /camera/image_raw)
# - tof_camera_node (reads /tof/depth/image_raw)
```

### Verification Commands (Terminal 3)

```bash
# 3. In Terminal 3: Verify all topics
source ~/ros2_moon_rover/install/setup.bash

# Check all topics are published
ros2 topic list
# Should show:
# /vex/odom_raw       ← From V5 Brain via serial bridge
# /imu/data           ← From MPU6050
# /camera/image_raw   ← From IMX500
# /tof/depth/image_raw ← From Arducam ToF

# Verify topic frequencies
ros2 topic hz /vex/odom_raw
# Should show: ~100 Hz (V5 encoder sampling rate)

ros2 topic hz /imu/data
# Should show: ~50 Hz

ros2 topic hz /camera/image_raw
# Should show: ~30 Hz

ros2 topic hz /tof/depth/image_raw
# Should show: ~30 Hz
```

### Real-Time Data Verification

```bash
# 4. Echo each topic to verify data
# Terminal 4:

# Check V5 encoder data (from serial bridge)
ros2 topic echo /vex/odom_raw --once
# Should show: x, y, theta, fl_ticks, fr_ticks, bl_ticks, br_ticks

# Check IMU data
ros2 topic echo /imu/data --once
# Should show: acceleration and gyroscope readings

# Check camera data
ros2 topic echo /camera/image_raw --once
# Should show: image metadata (height, width, encoding)

# Check depth data
ros2 topic echo /tof/depth/image_raw --once
# Should show: depth image metadata
```

### RViz Visualization

```bash
# 5. Open RViz to visualize sensor data
rviz2 -d ~/ros2_moon_rover/src/rover_launch/config/phase_1.rviz

# Or manually:
rviz2

# Add displays:
# 1. Image (Camera) → /camera/image_raw
# 2. Image (ToF) → /tof/depth/image_raw
# 3. Odometry → /vex/odom_raw
# 4. Axes → base_link frame (if available)
```

### Success Criteria ✅

- [x] Serial bridge connecting (Terminal 1 shows "Client initialized")
- [x] Phase 1 launch successful (Terminal 2 shows "sensors started")
- [x] 4 topics published (Terminal 3 ros2 topic list)
- [x] 4 topic frequencies correct (Terminal 3 ros2 topic hz)
- [x] All data values realistic (Terminal 4 ros2 topic echo)
- [x] RViz displays showing live data

### Troubleshooting Phase 1

```bash
# Issue: Serial bridge won't connect
# Solution 1: Check /dev/ttyACM0 exists
ls -la /dev/ttyACM0

# Solution 2: Restart V5 Brain
# Solution 3: Check baud rate is 115200 (not 9600)

# Issue: /vex/odom_raw topic not appearing
# Solution: Check V5 terminal shows [ROS] Connected
pros terminal

# Issue: IMU not responding
# Solution: Check I2C address
i2cdetect -y 1
# Should show 68 in address list

# Issue: Camera not found
# Solution: Check device file
ls /dev/video0
# If missing: dmesg | grep usb

# Issue: Topics appear but no data
# Solution: Check hardware is powered
# - V5 Brain plugged in
# - Raspberry Pi powered
# - All sensors getting power
```

---

## 📋 PHASE 1 LAUNCH FILE CONTENT

### File: `src/rover_launch/launch/phase_1_sensors.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    rover_launch_dir = get_package_share_directory('rover_launch')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('vex_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_i2c_bus', default_value='1'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        
        # VEX Driver Node
        # (Note: Assumes rosserial bridge running separately)
        Node(
            package='vex_driver_node',
            executable='vex_driver_node',
            name='vex_driver',
            output='screen',
            parameters=[],
            remappings=[
                ('/vex/odom_raw', '/vex/odom_raw'),
            ]
        ),
        
        # IMU Driver Node
        Node(
            package='imu_driver_node',
            executable='imu_driver_node',
            name='imu_driver',
            output='screen',
            parameters=[
                {'i2c_bus': LaunchConfiguration('imu_i2c_bus')},
                {'i2c_address': 0x68},
            ],
            remappings=[
                ('/imu/data', '/imu/data'),
            ]
        ),
        
        # IMX500 Camera Driver Node
        Node(
            package='imx500_camera_node',
            executable='imx500_camera_node',
            name='imx500_camera',
            output='screen',
            parameters=[
                {'device': LaunchConfiguration('camera_device')},
                {'frame_rate': 30},
                {'resolution': '1280x720'},
            ],
            remappings=[
                ('/camera/image_raw', '/camera/image_raw'),
                ('/camera/camera_info', '/camera/camera_info'),
            ]
        ),
        
        # ToF Camera Driver Node
        Node(
            package='tof_camera_node',
            executable='tof_camera_node',
            name='tof_camera',
            output='screen',
            parameters=[
                {'i2c_bus': 1},
                {'sensor_width': 160},
                {'sensor_height': 120},
            ],
            remappings=[
                ('/tof/depth/image_raw', '/tof/depth/image_raw'),
            ]
        ),
    ])
```

---

## ✅ PHASES 2-11 OVERVIEW

Each subsequent phase builds on previous phases:

| Phase | Nodes Launched | Key Topics | Time |
|-------|---|---|---|
| **0** | None (verification) | - | 10 min |
| **1** | 4 sensor drivers | `/vex/odom_raw`, `/imu/data`, camera topics | 30 min |
| **2** | Static TF publisher | TF tree `/map`→`/base_link` | 15 min |
| **3** | EKF fusion node | `/odom` (fused odometry) | 20 min |
| **4** | RTAB-Map SLAM | `/rtabmap/mapData`, loop closure | 15 min |
| **5** | Terrain layer | `/local_costmap`, obstacles detected | 15 min |
| **6** | Auto-exposure controller | Camera exposure adjusted | 10 min |
| **7** | Perception (3 nodes) | `/ai_camera/classification`, `/landmarks/detections` | 15 min |
| **8** | Localization + Health | `/mission_health` monitoring | 15 min |
| **9** | Nav2 (4 nodes) | `/plan`, `/cmd_vel`, autonomous navigation | 30 min |
| **10** | Waypoint recorder | `/recorded_waypoints`, save mission | 20 min |
| **11** | Full mission | 30-minute autonomous run | 40 min |

---

## 🔌 SERIAL BRIDGE MANAGEMENT

### Starting Serial Bridge

```bash
# Option 1: Standalone (recommended for development)
ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

# Option 2: In launch file (advanced)
# Add to any phase's launch file after Phase 1
```

### Monitoring Serial Bridge

```bash
# Check if bridge is running
ps aux | grep serial_node
# Should show: ros2 run rosserial_python...

# Monitor topics flowing
ros2 topic list | grep vex
# Should show: /vex/odom_raw

# Check message rate
ros2 topic hz /vex/odom_raw
# Should show: ~100 Hz
```

### Troubleshooting Serial Bridge

```bash
# Issue: Bridge crashes
# Solution: Restart V5 Brain and reconnect USB

# Issue: Topics stop flowing
# Solution: Restart serial bridge
# Ctrl+C to stop current instance
# Run command above to restart

# Issue: Permission denied
# Solution: sudo usermod -a -G dialout $USER
# Then log out and back in

# Issue: Wrong port (/dev/ttyUSB0 vs /dev/ttyACM0)
# Solution: Check which port V5 is on
# ls /dev/ttyACM* or ls /dev/ttyUSB*
# Use correct port in launch file
```

---

## 📊 SERIAL DATA FLOW

```
V5 Brain (PROS firmware)
    ↓ [reads 4 encoders at real-time]
[Odometry calculation - differential drive]
    ↓ [formats as rover_msgs/VexRawOdometry]
[rosserial encoding]
    ↓ [USB serial at 115200 baud]
Linux PC
    ↓
rosserial_python serial bridge
    ↓
/vex/odom_raw topic (100 Hz)
    ↓
vex_driver_node subscribes
    ↓
Republishes as full Odometry message
    ↓
Downstream packages (EKF, Nav2, etc.)
```

---

## ⏱️ LAUNCH SEQUENCE TIMING

```
Time 0:00
├─ Terminal 1: Start serial bridge
│  └─ Wait for "Client initialized" (3-5 sec)
│
├─ Time 0:05
│  Terminal 2: Launch Phase 1
│  └─ Wait for all 4 nodes to start (5-10 sec)
│
├─ Time 0:15
│  Terminal 3: Verify topics (5 min)
│  └─ Check frequencies and data flow
│
├─ Time 0:20
│  Terminal 4: Echo real data (5 min)
│  └─ Confirm realistic values
│
├─ Time 0:25
│  Terminal 5: Open RViz (5 min)
│  └─ Visualize live sensor data

Total Phase 1 time: ~30 minutes setup + verification
```

---

## 🚨 CRITICAL NOTES

### Serial Bridge MUST Start First
❌ Wrong: `ros2 launch phase_1...` then `serial_node`  
✅ Correct: `serial_node` first, wait for connection, THEN launch

### Keep Serial Bridge Running
❌ Wrong: Close terminal running serial bridge  
✅ Correct: Keep terminal open throughout all phases

### Baud Rate Must Match
❌ Wrong: Using 9600 or 115200 inconsistently  
✅ Correct: All tools use 115200 baud (rosserial default)

### USB Port Consistency
❌ Wrong: Changing USB ports between runs  
✅ Correct: Always use same USB port (document it!)

---

## 📋 CHECKLIST FOR PHASE 1

Before proceeding to Phase 2:

```
Serial Communication:
□ /dev/ttyACM0 exists and readable
□ Serial bridge running in Terminal 1
□ V5 Brain shows "Connected" in pros terminal

Sensors Publishing:
□ /vex/odom_raw topic exists (100 Hz)
□ /imu/data topic exists (50 Hz)
□ /camera/image_raw topic exists (30 Hz)
□ /tof/depth/image_raw topic exists (30 Hz)

Data Verification:
□ VEX encoder values changing when wheels move
□ IMU showing realistic acceleration/gyro data
□ Camera showing image data
□ ToF showing depth values

RViz Visualization:
□ Image displays showing live camera/depth
□ Odometry visualization (if configured)
□ No errors in RViz console
```

---

## ✅ PHASE 1 SUCCESS

When Phase 1 complete, you have:
✅ Serial connection to V5 Brain established
✅ Motor encoders publishing odometry at 100 Hz
✅ All 4 sensors reading and publishing data
✅ Real-time data flowing into ROS 2
✅ Foundation for all subsequent phases

**Ready for Phase 2: Static TF Tree** 🚀

---

## 📈 NEXT PHASES (Brief Overview)

After Phase 1 succeeds:

**Phase 2:** Add static TF transforms (frame setup)
**Phase 3:** Add EKF fusion (smooth odometry)
**Phase 4:** Add SLAM (map building)
**Phase 5:** Add terrain detection (obstacle avoidance)
**Phase 6:** Add auto-exposure (camera tuning)
**Phase 7:** Add perception (classification + detection)
**Phase 8:** Add health monitoring (system diagnostics)
**Phase 9:** Add Nav2 (autonomous navigation)
**Phase 10:** Record waypoint missions
**Phase 11:** Deploy 30-minute autonomous mission

Each phase builds on previous. Do NOT skip ahead!

---

**Phase 1 with Serial Bridge is complete! Ready to launch!** 🌙
