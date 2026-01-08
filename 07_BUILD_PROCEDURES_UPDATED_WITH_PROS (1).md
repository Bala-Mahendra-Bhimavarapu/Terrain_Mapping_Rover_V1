# 07_BUILD_PROCEDURES_UPDATED_WITH_PROS.md
## Complete Build Guide: VEX V5 Brain PROS + ROS 2 Linux System
## Updated January 7, 2026 - Includes PROS Setup & Serial Configuration

**Status:** Complete step-by-step build guide  
**Total Steps:** 14 (4 for V5 Brain + 10 for Linux)  
**Build Time:** 2.5-3 hours total (can run in parallel)  
**Outcome:** Fully functional rover with motor odometry publishing to ROS 2  

---

## 🎯 OVERVIEW

This guide has **TWO parallel tracks:**

### **TRACK 1: VEX V5 Brain (Steps 0A-0D)** - 30 minutes
- Install PROS CLI
- Create PROS project
- Upload firmware to V5 Brain
- Test encoder reading

### **TRACK 2: ROS 2 Linux (Steps 1-10)** - 2 hours
- Install dependencies
- Create workspace
- Clone source packages
- Build all packages

**These can run simultaneously!**

---

## 🚀 STEP 0A: Prepare V5 Brain & Install PROS CLI

### Prerequisites
- VEX V5 Robot Brain (connected via USB)
- Computer with Python 3.7+
- Internet connection

### Installation

```bash
# 1. Install PROS CLI
pip3 install pros-cli

# 2. Verify installation
pros --version
# Should show: PROS CLI v5.x.x

# 3. Download PROS IDE (optional but recommended)
# VS Code → Extensions → Search "PROS"
# Install the official PROS extension

# 4. Connect V5 Brain via USB
# - Plug Micro-USB into V5 Brain back panel
# - Plug other end into computer USB port
# - Brain should appear in file manager

# 5. Verify connection
ls /dev/ttyACM*
# Should show: /dev/ttyACM0 (or similar)

# 6. Check V5 Brain firmware
pros info
# Should show: V5 target detected

# 7. Update firmware if needed
pros conduct upgrade
# Follow on-screen prompts

echo "[STEP 0A] ✅ PROS CLI installed and V5 Brain connected"
```

### Troubleshooting
```bash
# If PROS not found after pip install
# Add to PATH:
export PATH="$HOME/.local/bin:$PATH"

# If V5 doesn't appear as /dev/ttyACM0
# Restart the brain and reconnect

# If permission denied on /dev/ttyACM0
sudo usermod -a -G dialout $USER
# Log out and back in
```

---

## 🔧 STEP 0B: Create PROS Project & Add rosserial

### Create New Project

```bash
# 1. Create PROS project
cd ~/
pros conduct new-project vex_ros_brain
cd vex_ros_brain

# 2. Verify project structure
ls -la
# Should show:
# include/
# src/
# project.yaml
# Makefile

# 3. Select V5 as target
pros target select v5
# When prompted, choose "V5"

# 4. Set project name in project.yaml
cat project.yaml
# Should have: target: v5

# 5. Create main.cpp if not present
# (Usually auto-created by PROS)

echo "[STEP 0B Part 1] ✅ PROS project created"
```

### Add rosserial Library

```bash
# 1. Fetch rosserial library for VEX V5
pros c fetch rosserial_vex_v5

# 2. Apply library to project
pros c apply rosserial_vex_v5

# 3. Verify library installed
ls -la include/
# Should include rosserial headers

# 4. Build to verify everything works
pros build
# Should complete without errors

echo "[STEP 0B Part 2] ✅ rosserial library added"
```

---

## 📝 STEP 0C: Copy PROS Code & Upload to V5 Brain

### Copy Source Files

```bash
# 1. From 02_VEX_DRIVER_WITH_PROS_CODE.md, copy src/robot.cpp
# Location: ~/vex_ros_brain/src/robot.cpp

# 2. From same file, copy include/robot-config.h
# Location: ~/vex_ros_brain/include/robot-config.h

# 3. Verify files exist
ls -la src/robot.cpp
ls -la include/robot-config.h
# Both should show green checkmarks

# 4. Verify file content has motor definitions
grep "motor_FL" include/robot-config.h
# Should show: extern pros::Motor motor_FL;

echo "[STEP 0C Part 1] ✅ Source files copied"
```

### Build PROS Project

```bash
# 1. Build the project
pros build
# Should show:
# [INFO] Linking project
# [INFO] Build completed successfully

# 2. Check build output
ls -la bin/
# Should have: output.elf, output.bin, etc.

# 3. If build fails, check errors
# Most common: Missing includes or library issues
# Solution: Verify robot-config.h and src/robot.cpp copied correctly

echo "[STEP 0C Part 2] ✅ Project built successfully"
```

### Upload to V5 Brain

```bash
# 1. Ensure V5 Brain connected via USB
ls -la /dev/ttyACM0

# 2. Upload firmware
pros upload
# Will prompt: "Select target device"
# Choose: V5 Brain (should auto-detect)

# 3. Wait for upload to complete
# Should show: [INFO] Upload complete

# 4. V5 Brain screen will show loading bar
# Wait until complete (30-60 seconds)

# 5. Brain will reboot - wait 10 seconds

echo "[STEP 0C Part 3] ✅ Firmware uploaded to V5 Brain"
```

---

## ✅ STEP 0D: Test V5 Brain Serial Connection & Encoders

### Monitor V5 Brain Console

```bash
# 1. Open serial terminal to V5 Brain
pros terminal

# 2. You should see:
# [STARTUP] VEX V5 Brain initializing...
# [INIT] Motors initialized and encoders reset
# [ROS] Initialized - waiting for connection...

# 3. Verify motors are detected
# Check that ports 1-4 show motors

# 4. Test encoder reading (manual test)
# Manually spin each wheel by hand
# Watch console for changing tick values

# 5. Verify 100 Hz publishing
# Count messages per second
# Should see ~100 messages/sec when ready

# 6. Exit terminal: Ctrl+C

echo "[STEP 0D Part 1] ✅ V5 Brain console verified"
```

### Test USB Serial Connection

```bash
# 1. Check device file exists
ls -la /dev/ttyACM0
# Should show: crw-rw-rw- (or similar)

# 2. Test serial communication
# Install screen (if not available)
sudo apt-get install screen

# 3. Connect to V5 at 115200 baud
screen /dev/ttyACM0 115200

# 4. You should see console output from V5
# [STARTUP] VEX V5 Brain initializing...

# 5. Exit screen: Ctrl+A, then Ctrl+D

# 6. If permission denied:
sudo usermod -a -G dialout $USER
# Then log out and back in

echo "[STEP 0D Part 2] ✅ USB serial connection verified"
```

---

## 1️⃣ STEP 1: Install ROS 2 Dependencies (Linux)

### System Requirements
- Ubuntu 20.04 or 22.04 (ROS 2 compatible)
- 8GB RAM minimum
- 10GB free disk space
- Internet connection

### Install Dependencies

```bash
# 1. Update system
sudo apt update
sudo apt upgrade -y

# 2. Install build tools
sudo apt install -y build-essential cmake git wget curl \
  python3-pip python3-dev python3-colcon-common-extensions

# 3. Install ROS 2 Humble (if not already installed)
# Follow official ROS 2 installation: https://docs.ros.org/en/humble/

# 4. Install SLAM & Nav2 dependencies
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rtabmap-ros \
  ros-humble-rviz2 \
  ros-humble-tf2 \
  ros-humble-tf2-ros

# 5. Install serial communication
sudo apt install -y \
  ros-humble-rosserial \
  ros-humble-rosserial-python \
  python-is-python3

# 6. Verify ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 --version
# Should show: ROS 2 (humble)

echo "[STEP 1] ✅ Dependencies installed"
```

---

## 2️⃣ STEP 2: Create ROS 2 Workspace & Folder Structure

```bash
# 1. Create workspace directory
mkdir -p ~/ros2_moon_rover/src
cd ~/ros2_moon_rover

# 2. Create all 18 custom package directories
mkdir -p src/rover_msgs
mkdir -p src/vex_driver_node
mkdir -p src/imu_driver_node
mkdir -p src/imx500_camera_node
mkdir -p src/tof_camera_node
mkdir -p src/static_tf_publisher
mkdir -p src/ekf_fusion_node
mkdir -p src/imx500_classifier_node
mkdir -p src/auto_exposure_controller_node
mkdir -p src/yolo_detector_node
mkdir -p src/landmark_map_server
mkdir -p src/mission_health_monitor
mkdir -p src/landmark_verification_node
mkdir -p src/waypoint_recorder_node
mkdir -p src/waypoint_player_node
mkdir -p src/mission_logger_node
mkdir -p src/streaming_node
mkdir -p src/rover_launch

# 3. Create subdirectories for each package
for pkg in src/*/; do
  mkdir -p "$pkg/src"
  mkdir -p "$pkg/config"
  mkdir -p "$pkg/launch"
done

# 4. Verify structure
find src -type d | head -20
# Should show nested structure

# 5. Create build directories
mkdir -p build install log

# 6. Source ROS 2
source /opt/ros/humble/setup.bash

echo "[STEP 2] ✅ Workspace and folder structure created"
```

---

## 3️⃣ STEP 3: Clone Source Packages

```bash
# 1. Change to src directory
cd ~/ros2_moon_rover/src

# 2. Clone robot_localization (for EKF)
git clone -b humble https://github.com/cra-ros-pkg/robot_localization.git

# 3. Clone navigation2
git clone -b humble https://github.com/ros-planning/navigation2.git
git clone -b humble https://github.com/ros-planning/nav2_msgs.git

# 4. Clone rtabmap_ros
git clone -b humble https://github.com/introlab/rtabmap_ros.git

# 5. Clone image_common (for cameras)
git clone -b humble https://github.com/ros-perception/image_common.git

# 6. Clone diagnostics (for health monitoring)
git clone -b humble https://github.com/ros/diagnostics.git

# 7. Verify all cloned
ls -la | grep git
# Should show 6+ directories

echo "[STEP 3] ✅ Source packages cloned"
```

---

## 4️⃣ STEP 4: Copy Implementation Files from Documentation

```bash
# 1. From 01_ROVER_MSGS.md
# Copy message definitions to: src/rover_msgs/

# 2. From 02_VEX_DRIVER.md
# Copy C++ code to: src/vex_driver_node/src/
# Copy CMakeLists.txt and package.xml

# 3. From 03_IMU_DRIVER.md
# Copy to: src/imu_driver_node/

# 4. From 04_CAMERA_AND_TOF_DRIVERS.md
# Copy IMX500 to: src/imx500_camera_node/
# Copy ToF to: src/tof_camera_node/

# 5. From 05_STATIC_TF_PUBLISHER.md
# Copy to: src/static_tf_publisher/

# 6. From 06_EKF_FUSION_NODE.md
# Copy to: src/ekf_fusion_node/
# Copy YAML: src/rover_launch/config/ekf_params.yaml

# 7. From 07_PERCEPTION_NODES.md
# Copy 3 packages to their directories

# 8. From 08_LOCALIZATION_NODES.md
# Copy 3 packages to their directories

# 9. From 09_NAVIGATION_NODES.md
# Copy 4 packages to their directories

# 10. From 11_CONFIGURATION_REFERENCE.md
# Copy all YAML files to: src/rover_launch/config/

# 11. From 06_LAUNCH_FILES.md
# Copy all launch files to: src/rover_launch/launch/

# 12. Verify files copied
find src -name "*.cpp" | wc -l
# Should show 10+ cpp files

echo "[STEP 4] ✅ Implementation files copied"
```

---

## 5️⃣ STEP 5: Build rover_msgs FIRST (CRITICAL!)

```bash
# 1. IMPORTANT: Build rover_msgs first - it's a dependency for all other packages
cd ~/ros2_moon_rover

# 2. Build only rover_msgs
colcon build --packages-select rover_msgs

# 3. Check for success
# Should show: [INFO] colcon build: Packages processed: 1
#               [INFO] Summary: 1 package finished successfully

# 4. If build fails, check:
# - package.xml exists in src/rover_msgs/
# - Message files in src/rover_msgs/msg/
# - CMakeLists.txt is correct

# 5. Source the build
source install/setup.bash

echo "[STEP 5] ✅ rover_msgs built successfully"
```

---

## 6️⃣ STEP 6: Build Source Packages

```bash
# 1. Build cloned source packages
cd ~/ros2_moon_rover
colcon build --packages-select \
  robot_localization \
  navigation2 \
  nav2_msgs \
  rtabmap_ros \
  image_common \
  diagnostics

# 2. This will take 10-20 minutes
# Wait for completion

# 3. Check for errors
# Should show: Summary: X packages finished successfully

# 4. If any fail, check:
# - Missing dependencies: rosdep install --from-path src --ignore-src -r -y
# - Disk space: df -h
# - Compiler errors in output

echo "[STEP 6] ✅ Source packages built"
```

---

## 7️⃣ STEP 7: Build All Custom Packages

```bash
# 1. Build all remaining packages
cd ~/ros2_moon_rover
colcon build

# 2. This will take 20-30 minutes
# Monitor output for errors

# 3. Expected output:
# Summary: 23 packages finished successfully

# 4. If any package fails:
# Rebuild just that package:
# colcon build --packages-select package_name

# 5. Check for build output
ls -la build/*/
# Should show multiple package directories

echo "[STEP 7] ✅ All custom packages built"
```

---

## 8️⃣ STEP 8: Source Workspace & Verify Build

```bash
# 1. Source the workspace
cd ~/ros2_moon_rover
source install/setup.bash

# 2. Add to ~/.bashrc for automatic sourcing
echo "source ~/ros2_moon_rover/install/setup.bash" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 3. Verify all packages available
ros2 pkg list | grep -E "(vex|imu|ekf|perception|localization|navigation)" | wc -l
# Should show: 18+ packages

# 4. Check package paths
ros2 pkg prefix vex_driver_node
# Should show: ~/ros2_moon_rover/install/vex_driver_node

# 5. Verify executables
ros2 pkg executables vex_driver_node
# Should show available executables

echo "[STEP 8] ✅ Workspace sourced and verified"
```

---

## 9️⃣ STEP 9: Verify Build Completeness

```bash
# 1. Check all 23 packages built
ls install/ | wc -l
# Should show: 23+ (includes ROS 2 packages)

# 2. Verify critical packages
for pkg in rover_msgs vex_driver_node imu_driver_node ekf_fusion_node; do
  if [ -d "install/$pkg" ]; then
    echo "✅ $pkg exists"
  else
    echo "❌ $pkg missing"
  fi
done

# 3. Check for compilation issues
grep -r "error:" build/ 2>/dev/null
# Should return nothing (no errors)

# 4. Verify executable permissions
find install -name "*.so" -o -name "*_node" | head -5
# Should show shared libraries and executables

# 5. Test package imports
python3 -c "import numpy; print('✅ Python dependencies OK')"

echo "[STEP 9] ✅ Build verification complete"
```

---

## 🔟 STEP 10: Test Individual Packages & Serial Connection

### Test VEX Driver with Serial Connection

```bash
# 1. Ensure V5 Brain connected via USB
ls -la /dev/ttyACM0
# Must show device file

# 2. Source workspace
source install/setup.bash

# 3. Start rosserial serial bridge
ros2 run rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

# 4. In another terminal, verify topic:
source install/setup.bash
ros2 topic list | grep vex
# Should show: /vex/odom_raw

# 5. Echo the topic to see data:
ros2 topic echo /vex/odom_raw
# Should show encoder values updating at 100 Hz

# 6. Check message frequency:
ros2 topic hz /vex/odom_raw
# Should show: average rate: 100.00 Hz

# 7. If topic not appearing:
# - Check V5 Brain console: pros terminal
# - Verify serial connection: ls -la /dev/ttyACM0
# - Check baud rate matches (115200)
```

### Test Other Key Packages

```bash
# 1. Test TF broadcaster
ros2 run static_tf_publisher static_transform_publisher --help

# 2. Test EKF fusion
ros2 run ekf_fusion_node ekf_fusion_node --help

# 3. Test IMU driver
ros2 run imu_driver_node imu_driver_node --help

# 4. If any fail to run:
# - Verify package built: colcon build --packages-select package_name
# - Check for missing dependencies
# - Review build output for warnings

echo "[STEP 10] ✅ Individual package tests complete"
```

---

## 📊 VERIFICATION CHECKLIST

After completing all steps:

```bash
# ✅ V5 Brain (Steps 0A-0D)
□ PROS CLI installed
□ PROS project created
□ Code uploaded to V5 Brain
□ Serial connection verified at 115200 baud
□ Encoders reading correctly
□ 100 Hz publishing rate achieved

# ✅ Linux Build (Steps 1-10)
□ ROS 2 dependencies installed
□ Workspace created with 18 folders
□ Source packages cloned (6 packages)
□ Implementation files copied
□ rover_msgs built first
□ Source packages built
□ All 23 packages built successfully
□ Workspace sourced
□ /vex/odom_raw topic receiving data at 100 Hz
□ All executables verified

# ✅ Serial Communication
□ /dev/ttyACM0 exists and readable
□ rosserial bridge running
□ V5 Brain connected to Linux PC
□ Data flowing from V5 to ROS 2
□ Message frequency at 100 Hz
```

---

## 🚨 TROUBLESHOOTING

### Issue: V5 doesn't appear as /dev/ttyACM0
```bash
# Solution 1: Restart brain
# - Hold brain button for 10 seconds
# - Reconnect USB

# Solution 2: Check permissions
ls -la /dev/ttyACM*
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Issue: "Package not found" during build
```bash
# Solution: Install missing dependencies
rosdep install --from-path src --ignore-src -r -y
colcon build
```

### Issue: /vex/odom_raw topic not appearing
```bash
# Solution 1: Check V5 Brain console
pros terminal
# Should show: [ROS] Connected...

# Solution 2: Verify serial bridge running
ps aux | grep serial_node
# Should show running process

# Solution 3: Check baud rate
# Verify both V5 and bridge use 115200 baud
```

### Issue: Build fails with C++ errors
```bash
# Solution: Check that implementation files copied correctly
# - Verify src/robot.cpp exists in V5 PROS project
# - Verify CMakeLists.txt in each ROS 2 package
# - Check for syntax errors in copied code
```

---

## 📈 BUILD TIME SUMMARY

| Step | Time | Status |
|------|------|--------|
| 0A: PROS CLI | 10 min | Setup |
| 0B: Create project | 5 min | Setup |
| 0C: Upload code | 10 min | Upload |
| 0D: Test | 5 min | Verification |
| **V5 Total** | **30 min** | ✅ |
| 1: Dependencies | 15 min | Install |
| 2: Workspace | 5 min | Setup |
| 3: Clone packages | 10 min | Download |
| 4: Copy files | 15 min | Copy |
| 5: Build rover_msgs | 5 min | Build |
| 6: Build source | 15 min | Build |
| 7: Build custom | 30 min | Build |
| 8: Verify | 5 min | Check |
| 9: Complete check | 5 min | Check |
| 10: Test | 10 min | Test |
| **Linux Total** | **2 hours** | ✅ |
| **TOTAL (parallel)** | **2.5-3 hours** | ✅ Complete |

---

## ✅ WHAT'S NEXT

After completing all 10 steps:

1. **Verify everything built:** `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`
2. **Connect V5 Brain via USB:** Micro-USB port to Linux PC
3. **Launch Phase 1:** `ros2 launch rover_launch phase_1_sensors.launch.py`
4. **Verify /vex/odom_raw:** `ros2 topic echo /vex/odom_raw`
5. **Follow 06_LAUNCH_FILES_UPDATED.md** for Phase 2-11

---

## 🌙 BUILD COMPLETE!

You now have:
✅ VEX V5 Brain with PROS firmware (motor odometry)
✅ ROS 2 Linux system with 23 packages
✅ Serial communication (USB, 115200 baud, 100 Hz)
✅ All drivers and nodes ready to test
✅ Complete verification procedures

**Ready for Phase 1 launch and testing!** 🚀
