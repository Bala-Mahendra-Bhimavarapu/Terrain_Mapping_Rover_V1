<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# whenever i run a command that involves the vex brain what is the exact line i have to right before to serial connect to the vex v5 brain

Perfect! I found exactly what you need. Here's the **exact command** you must run FIRST before any VEX Brain commands:

***

## 🔌 **SERIAL BRIDGE COMMAND (MUST RUN FIRST!)**

### **The Exact Line:**

```bash
ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```


***

## 📋 **COMPLETE WORKFLOW**

### **Every time you want to use VEX Brain with ROS 2:**

**Terminal 1 (ALWAYS START HERE):**

```bash
# Step 1: Source your workspace
source ~/ros2_moon_rover/install/setup.bash

# Step 2: Start the serial bridge (THIS IS THE LINE!)
ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

# You should see:
# [INFO] ROS Serial Python Node
# [INFO] Connecting to /dev/ttyACM0 at 115200 baud
# [INFO] Requesting topics...
# [INFO] Note: publish buffer size is 512 bytes
# [INFO] Setup service server on /vex/odom_raw [rover_msgs/VexOdometry]
```

**Terminal 2 (THEN run your commands):**

```bash
# Source workspace again
source ~/ros2_moon_rover/install/setup.bash

# Now you can run any ROS 2 commands that need VEX data:
ros2 topic list
ros2 topic echo /vex/odom_raw
ros2 launch rover_launch phase_1_sensors.launch.py
```


***

## 🔍 **WHERE THIS IS DOCUMENTED**

From **07_BUILD_PROCEDURES_UPDATED_WITH_PROS.md**, Step 10:[^1]

> **STEP 10: Test VEX Driver with Serial Connection**
>
> ```bash > 3. Start rosserial serial bridge > ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200 >  > 4. In another terminal, verify topic > source install/setup.bash > ros2 topic list | grep vex > Should show /vex/odom_raw > ```

***

## ⚠️ **IMPORTANT NOTES**

### **Port Name:**

- **✅ Correct:** `/dev/ttyACM0` (VEX V5 Brain uses ACM, not USB)
- **❌ Wrong:** `/dev/ttyUSB0` (this won't work)


### **Baud Rate:**

- **✅ Must be:** `115200` (both on V5 Brain firmware and serial_node.py)


### **Before Running:**

```bash
# Verify V5 Brain is connected
ls -la /dev/ttyACM0

# Should show:
# crw-rw-rw- 1 root dialout ... /dev/ttyACM0

# If permission denied, add yourself to dialout group:
sudo usermod -a -G dialout $USER
# Then log out and back in
```


***

## 📝 **FULL EXAMPLE SESSION**

```bash
# Terminal 1: Serial Bridge (LEAVE THIS RUNNING!)
user@rover:~$ source ~/ros2_moon_rover/install/setup.bash
user@rover:~$ ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
[INFO] [1736386800.123456789] [ros.rosserial_python]: ROS Serial Python Node
[INFO] [1736386800.234567890] [ros.rosserial_python]: Connecting to /dev/ttyACM0 at 115200 baud
[INFO] [1736386800.345678901] [ros.rosserial_python]: Requesting topics...
[INFO] [1736386800.456789012] [ros.rosserial_python]: Note: publish buffer size is 512 bytes
[INFO] [1736386800.567890123] [ros.rosserial_python]: Setup publisher on /vex/odom_raw [rover_msgs/VexOdometry]
# ⬆️ Bridge is now active! Keep this terminal open.


# Terminal 2: Run your commands
user@rover:~$ source ~/ros2_moon_rover/install/setup.bash
user@rover:~$ ros2 topic list
/parameter_events
/rosout
/vex/odom_raw    # ✅ V5 Brain is publishing!

user@rover:~$ ros2 topic echo /vex/odom_raw
header:
  stamp:
    sec: 1736386801
    nanosec: 123456789
  frame_id: base_link
motor_fl_ticks: 1523
motor_fr_ticks: 1498
motor_bl_ticks: 1510
motor_br_ticks: 1505
linear_velocity: 0.15
angular_velocity: 0.0
---
# ✅ Data is flowing from V5 Brain!

user@rover:~$ ros2 launch rover_launch phase_1_sensors.launch.py
# ✅ Sensors launch successfully!
```


***

## 🚀 **QUICK REFERENCE CARD**

### **Every Time You Use VEX Brain:**

| Step | Command | Terminal |
| :-- | :-- | :-- |
| **1** | `source ~/ros2_moon_rover/install/setup.bash` | Terminal 1 |
| **2** | `ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200` | Terminal 1 (leave open) |
| **3** | `source ~/ros2_moon_rover/install/setup.bash` | Terminal 2 |
| **4** | Run your ROS 2 commands | Terminal 2 |


***

## ✅ **ANSWER SUMMARY**

**The exact line you must run BEFORE any VEX Brain command:**

```bash
ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```

**Why?** This creates a serial bridge between the V5 Brain (which speaks rosserial protocol) and ROS 2 (which expects native ROS 2 topics). Without it, `/vex/odom_raw` will never appear and your sensors won't work.[^1]

**Pro Tip:** Keep this running in a dedicated terminal window. If you close it, all VEX Brain communication stops immediately! 🔌🚀
<span style="display:none">[^2][^3]</span>

<div align="center">⁂</div>

[^1]: 07_BUILD_PROCEDURES_UPDATED_WITH_PROS-1.md

[^2]: 07_BUILD_PROCEDURES_UPDATED.md

[^3]: 06_LAUNCH_FILES.md

