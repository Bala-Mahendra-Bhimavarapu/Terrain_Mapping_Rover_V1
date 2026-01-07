# 07_BUILD_PROCEDURES.md - Complete Step-by-Step Build Guide
# Updated January 6, 2026 - All 23 Packages, Proper Folder Structure, Verified Consistency

---

## 🎯 PURPOSE

This guide provides **exact step-by-step instructions** to build your complete ROS 2 moon rover system from scratch on Raspberry Pi 5 with Bookworm OS.

**Follow every command exactly as written. Do not skip steps.**

---

## ⚙️ PREREQUISITES (DO THIS FIRST)

### Hardware Required
- [ ] Raspberry Pi 5 (8GB recommended)
- [ ] Bookworm OS installed
- [ ] Internet connection
- [ ] ~20GB free disk space
- [ ] 5V power supply (5A+)

### Software Required
- [ ] ROS 2 Humble installed
- [ ] colcon build tool
- [ ] Git installed
- [ ] Python 3.10+

### Verify Prerequisites
```bash
# Check ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 --version

# Check colcon
colcon --version

# Check git
git --version

# If any of these fail, install missing packages:
# ROS 2: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
# colcon: sudo apt install python3-colcon-common-extensions
# git: sudo apt install git
```

---

## 📋 STEP-BY-STEP BUILD PROCEDURES

### **STEP 1: Prepare Your System (5 minutes)**

#### 1.1 Source ROS 2 Environment
```bash
source /opt/ros/humble/setup.bash
```

**Verify:**
```bash
echo $ROS_DISTRO  # Should print: humble
```

#### 1.2 Install Additional Dependencies
```bash
sudo apt update
sudo apt install -y \
  python3-pip \
  python3-dev \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  ros-humble-diagnostic-aggregator \
  ros-humble-diagnostics \
  ros-humble-image-common \
  libserial-dev
```

**Verify:**
```bash
which colcon  # Should show: /usr/bin/colcon
```

---

### **STEP 2: Create Your Workspace (5 minutes)**

#### 2.1 Create Directory Structure
```bash
mkdir -p ~/ros2_moon_rover/src
cd ~/ros2_moon_rover
```

**Verify:**
```bash
ls -la
# Should show: build/ (may not exist yet), src/, log/ (may not exist yet)
```

#### 2.2 Create Subdirectories for Organization
```bash
cd ~/ros2_moon_rover/src

# Create package directories (we'll populate these later)
mkdir -p rover_msgs/{src,msg,include,launch,config}
mkdir -p vex_driver_node/{src,include,launch,config}
mkdir -p imu_driver_node/{src,include,launch,config}
mkdir -p imx500_camera_node/{src,launch,config}
mkdir -p tof_camera_node/{src,launch,config}
mkdir -p static_tf_publisher/{src,include,launch,config}
mkdir -p ekf_fusion_node/{src,launch,config}
mkdir -p imx500_classifier_node/{src,launch,config}
mkdir -p auto_exposure_controller_node/{src,launch,config}
mkdir -p yolo_detector_node/{src,launch,config}
mkdir -p landmark_map_server/{src,launch,config}
mkdir -p mission_health_monitor/{src,launch,config}
mkdir -p landmark_verification_node/{src,launch,config}
mkdir -p waypoint_recorder_node/{src,launch,config}
mkdir -p waypoint_player_node/{src,launch,config}
mkdir -p mission_logger_node/{src,launch,config}
mkdir -p streaming_node/{src,launch,config}
mkdir -p rover_launch/{launch,config}
```

**Verify:**
```bash
cd ~/ros2_moon_rover/src
ls -d */  # Should show all 18 custom packages
```

---

### **STEP 3: Clone Source Packages (10 minutes)**

These are the **source packages** that provide core ROS 2 functionality. We need to clone them into our workspace.

#### 3.1 Clone robot_localization (for EKF)
```bash
cd ~/ros2_moon_rover/src
git clone --branch humble https://github.com/cra-ros-pkg/robot_localization.git
```

**Verify:**
```bash
ls -la robot_localization/ | head -20
# Should show: CMakeLists.txt, package.xml, src/, etc.
```

#### 3.2 Clone navigation2 Stack
```bash
cd ~/ros2_moon_rover/src
git clone --branch humble https://github.com/ros-planning/navigation2.git
```

**Verify:**
```bash
ls navigation2/  # Should show: nav2_msgs/, nav2_core/, etc.
```

#### 3.3 Clone navigation2_msgs
```bash
cd ~/ros2_moon_rover/src
git clone --branch humble https://github.com/ros-planning/navigation2_msgs.git
```

**Verify:**
```bash
ls navigation2_msgs/
```

#### 3.4 Clone image_common (for camera drivers)
```bash
cd ~/ros2_moon_rover/src
git clone --branch humble https://github.com/ros-perception/image_common.git
```

#### 3.5 Clone diagnostics (for health monitoring)
```bash
cd ~/ros2_moon_rover/src
git clone --branch humble https://github.com/ros-perception/diagnostics.git
```

#### 3.6 Clone RTAB-Map (for SLAM)
```bash
cd ~/ros2_moon_rover/src
git clone --branch humble https://github.com/introlab/rtabmap.git
git clone --branch humble https://github.com/ros-perception/rtabmap_ros.git
```

**Verify All Clones:**
```bash
cd ~/ros2_moon_rover/src
ls -d */ | wc -l  # Should be: 25 (18 custom + 7 source)
```

---

### **STEP 4: Copy Custom Package Files (30 minutes)**

Now you'll copy files from the implementation documents into your package directories.

#### 4.1 Copy rover_msgs Package
From **01_ROVER_MSGS.md**, copy these files:

**Location:** `~/ros2_moon_rover/src/rover_msgs/`

**Files to copy:**
- `package.xml` → `rover_msgs/package.xml`
- `CMakeLists.txt` → `rover_msgs/CMakeLists.txt`
- Message files → `rover_msgs/msg/` (all 8 .msg files)

```bash
# Create msg directory structure
mkdir -p ~/ros2_moon_rover/src/rover_msgs/msg

# You'll copy the content from 01_ROVER_MSGS.md into these files
# Make sure to copy package.xml, CMakeLists.txt, and all message definitions
```

**Verify:**
```bash
ls ~/ros2_moon_rover/src/rover_msgs/
# Should show: CMakeLists.txt, package.xml, msg/
ls ~/ros2_moon_rover/src/rover_msgs/msg/
# Should show: 8 message files (.msg)
```

#### 4.2 Copy Driver Packages (VEX, IMU, Cameras, TF)

From **02_VEX_DRIVER.md**, **03_IMU_DRIVER.md**, **04_CAMERA_AND_TOF_DRIVERS.md**, **05_STATIC_TF_PUBLISHER.md**:

For each package:
```bash
# Example for vex_driver_node (repeat for other drivers)
# Copy from 02_VEX_DRIVER.md:
# - package.xml → ~/ros2_moon_rover/src/vex_driver_node/package.xml
# - CMakeLists.txt → ~/ros2_moon_rover/src/vex_driver_node/CMakeLists.txt
# - src/*.cpp → ~/ros2_moon_rover/src/vex_driver_node/src/
# - config/*.yaml → ~/ros2_moon_rover/src/vex_driver_node/config/
# - launch/*.py → ~/ros2_moon_rover/src/vex_driver_node/launch/
```

**Packages to copy:**
1. **vex_driver_node** from 02_VEX_DRIVER.md
2. **imu_driver_node** from 03_IMU_DRIVER.md
3. **imx500_camera_node** from 04_CAMERA_AND_TOF_DRIVERS.md
4. **tof_camera_node** from 04_CAMERA_AND_TOF_DRIVERS.md
5. **static_tf_publisher** from 05_STATIC_TF_PUBLISHER.md

**Verify Each Package:**
```bash
# For each package:
ls ~/ros2_moon_rover/src/[package_name]/
# Should show: CMakeLists.txt, package.xml, src/, config/, launch/

# Check C++ source files exist
ls ~/ros2_moon_rover/src/vex_driver_node/src/
# Should show: .cpp files
```

#### 4.3 Copy Fusion & Perception Packages

From **06_EKF_FUSION_NODE.md**, **07_PERCEPTION_NODES.md**:

**Packages to copy:**
1. **ekf_fusion_node** from 06_EKF_FUSION_NODE.md
   - Source code (C++)
   - Configuration (YAML with 200+ lines of tuning)
   - Launch files
   
2. **imx500_classifier_node** from 07_PERCEPTION_NODES.md (Package 1)
3. **auto_exposure_controller_node** from 07_PERCEPTION_NODES.md (Package 2)
4. **yolo_detector_node** from 07_PERCEPTION_NODES.md (Package 3)

```bash
# Verify structure
for pkg in ekf_fusion_node imx500_classifier_node auto_exposure_controller_node yolo_detector_node; do
  echo "=== $pkg ==="
  ls ~/ros2_moon_rover/src/$pkg/
done
```

#### 4.4 Copy Localization & Mission Packages

From **08_LOCALIZATION_NODES.md**, **09_NAVIGATION_NODES.md**:

**Packages to copy:**
1. **landmark_map_server** from 08_LOCALIZATION_NODES.md (Package 1)
2. **mission_health_monitor** from 08_LOCALIZATION_NODES.md (Package 2)
3. **landmark_verification_node** from 08_LOCALIZATION_NODES.md (Package 3)
4. **waypoint_recorder_node** from 09_NAVIGATION_NODES.md (Package 1)
5. **waypoint_player_node** from 09_NAVIGATION_NODES.md (Package 2)
6. **mission_logger_node** from 09_NAVIGATION_NODES.md (Package 3)
7. **streaming_node** from 09_NAVIGATION_NODES.md (Package 4)

```bash
# Verify all packages present
cd ~/ros2_moon_rover/src
ls -d */ | wc -l
# Should be: 25 (18 custom + 7 source packages)
```

#### 4.5 Copy Launch Files & Configuration

From **06_LAUNCH_FILES.md** and **11_CONFIGURATION_REFERENCE.md**:

**Copy launch files to:**
```bash
~/ros2_moon_rover/src/rover_launch/launch/
# Should contain 12 launch files:
# - phase_1_sensors.launch.py
# - phase_2_tf.launch.py
# - phase_3_ekf.launch.py
# - ... (through phase_11)
```

**Copy configuration files to:**
```bash
~/ros2_moon_rover/src/rover_launch/config/
# Should contain:
# - ekf_params.yaml (from 06_EKF_FUSION_NODE.md or 11_CONFIGURATION_REFERENCE.md)
# - rtabmap_params.yaml (from 11_CONFIGURATION_REFERENCE.md Section 1)
# - nav2_params.yaml (from 11_CONFIGURATION_REFERENCE.md Section 2)
# - static_transforms.yaml (from 05_STATIC_TF_PUBLISHER.md)
# - imx500_intrinsics.yaml (from 11_CONFIGURATION_REFERENCE.md Section 3)
# - tof_intrinsics.yaml (from 11_CONFIGURATION_REFERENCE.md Section 3)
# - imu_calibration.yaml (from 11_CONFIGURATION_REFERENCE.md Section 4)
# - vex_encoder_calibration.yaml (from 02_VEX_DRIVER.md or 11_CONFIGURATION_REFERENCE.md)
# - terrain_layer_params.yaml (from 11_CONFIGURATION_REFERENCE.md Section 5)
```

**Verify:**
```bash
ls ~/ros2_moon_rover/src/rover_launch/
# Should show: launch/, config/

ls ~/ros2_moon_rover/src/rover_launch/launch/ | wc -l
# Should be: 12 launch files

ls ~/ros2_moon_rover/src/rover_launch/config/ | wc -l
# Should be: 9+ config files
```

---

### **STEP 5: Build rover_msgs FIRST (Critical!) (5 minutes)**

**This MUST be built before any other custom package!**

```bash
cd ~/ros2_moon_rover
colcon build --packages-select rover_msgs --symlink-install
```

**Verify Success:**
```bash
# You should see:
# [0/1] Synchronizing sources in /home/pi/ros2_moon_rover/src
# [1/1] Completed 'rover_msgs'
#
# Summary: 1 package finished [Xms]

# Check that install/ directory was created
ls -la install/
# Should show: rover_msgs/, setup.bash, setup.sh, local_setup.bash
```

**If this fails:**
- Check that all 8 message files exist in `rover_msgs/msg/`
- Check that `package.xml` and `CMakeLists.txt` are present and valid
- Check for missing dependencies in `package.xml`

---

### **STEP 6: Build All Source Packages (10-15 minutes)**

Now build the source packages that provide core ROS 2 functionality.

```bash
cd ~/ros2_moon_rover
colcon build \
  --packages-select \
  robot_localization \
  navigation2_msgs \
  image_common \
  diagnostics \
  rtabmap \
  rtabmap_ros \
  --symlink-install
```

**Expected Output:**
```
[0/6] Synchronizing sources...
[1/6] Building 'robot_localization'
[2/6] Building 'navigation2_msgs'
...
Summary: 6 packages finished [time]
```

**Verify Success:**
```bash
ls install/ | grep -E "robot_localization|navigation2_msgs|rtabmap"
# Should show those packages
```

---

### **STEP 7: Build All Remaining Custom Packages (15-20 minutes)**

Now build all your custom rover packages.

```bash
cd ~/ros2_moon_rover
colcon build --symlink-install
```

**Expected Output:**
```
[0/18] Synchronizing sources...
[1/18] Building 'vex_driver_node'
[2/18] Building 'imu_driver_node'
...
Summary: 18 packages finished [time]
```

**If build fails:**

**Common Issues:**
1. **Missing dependencies** — Check package.xml for all required dependencies
2. **Missing message types** — Make sure rover_msgs was built first
3. **CMake errors** — Check CMakeLists.txt syntax
4. **Python import errors** — Check Python path and virtual environment

**Emergency Fix:**
```bash
# Build only one package to see detailed errors
colcon build --packages-select vex_driver_node --symlink-install
# Look at error message to fix issue

# Clean and retry
cd ~/ros2_moon_rover
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

### **STEP 8: Source Your Workspace (1 minute)**

After successful build, source your workspace.

```bash
cd ~/ros2_moon_rover
source install/setup.bash
```

**Make it permanent (optional):**
```bash
echo "source ~/ros2_moon_rover/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify:**
```bash
echo $CMAKE_PREFIX_PATH  # Should include ~/ros2_moon_rover/install
ros2 node list  # Should work (no nodes running yet, but command should work)
```

---

### **STEP 9: Verify Build Completeness (5 minutes)**

```bash
# Verify all packages built
cd ~/ros2_moon_rover/install
ls -d */ | wc -l
# Should be: 25+ (all custom + all source packages)

# Verify key packages
for pkg in rover_msgs vex_driver_node ekf_fusion_node imx500_classifier_node nav2_core rtabmap; do
  if [ -d "$pkg" ]; then
    echo "✓ $pkg present"
  else
    echo "✗ $pkg MISSING"
  fi
done
```

---

### **STEP 10: Test Individual Packages (10-20 minutes)**

Test that your build works by running individual nodes.

#### 10.1 Test VEX Driver (Check Connection)
```bash
cd ~/ros2_moon_rover
source install/setup.bash

# This will fail if VEX brain is not connected (expected)
timeout 5 ros2 run vex_driver_node vex_driver_node || echo "VEX not connected (expected)"
```

#### 10.2 Test IMU Driver (Check I2C Connection)
```bash
# This will fail if IMU is not on I2C bus (expected)
timeout 5 ros2 run imu_driver_node imu_driver_node || echo "IMU not connected (expected)"
```

#### 10.3 Test Static TF Broadcaster
```bash
# Run in one terminal:
ros2 run static_tf_publisher static_tf_broadcaster &

# Run in another terminal:
sleep 2
ros2 run tf2_tools view_frames.py

# You should see TF tree with: map → odom → base_link → {camera, tof, imu}
```

#### 10.4 Test EKF (Check Launch File)
```bash
# This tests that the launch file is valid
ros2 launch ekf_fusion_node ekf.launch.py 2>&1 | head -20

# You should see launch file starting (it will fail without sensors, but that's OK for now)
```

---

## ✅ BUILD COMPLETION CHECKLIST

Before proceeding to testing phases, verify:

- [ ] Step 1 complete — System dependencies installed
- [ ] Step 2 complete — Workspace created with correct structure
- [ ] Step 3 complete — All source packages cloned
- [ ] Step 4 complete — All custom package files copied
- [ ] Step 5 complete — rover_msgs built successfully
- [ ] Step 6 complete — Source packages built successfully
- [ ] Step 7 complete — All custom packages built successfully
- [ ] Step 8 complete — Workspace sourced (.bashrc updated)
- [ ] Step 9 complete — 25+ packages present in install/
- [ ] Step 10 complete — Individual package tests passed

**If any step failed:**
- Read error messages carefully
- Check file paths and names match exactly
- Verify files were copied correctly from documents
- Check that all dependencies are installed
- Try emergency fix: clean build and retry

---

## 🚀 NEXT STEP: PHASE TESTING

Once build is complete, proceed to **06_LAUNCH_FILES.md** to:
1. Launch Phase 1 sensors
2. Test each phase sequentially
3. Verify topics and TF tree
4. Calibrate sensors

---

## 🆘 EMERGENCY TROUBLESHOOTING

### Build Failed with "Package Not Found"
```bash
# Solution: Make sure you sourced the setup script
source /opt/ros/humble/setup.bash
source ~/ros2_moon_rover/install/setup.bash

# Then try building again
colcon build --symlink-install
```

### "Could not find package.xml"
```bash
# Solution: Make sure all custom package directories have package.xml and CMakeLists.txt
# Check:
find ~/ros2_moon_rover/src -maxdepth 2 -name "package.xml" | wc -l
# Should be: 25 or more

# If missing, copy from implementation documents
```

### Build Hangs or Takes Too Long
```bash
# Solution: Use different build options
colcon build --symlink-install --executor sequential

# Or build fewer packages at once
colcon build --packages-select rover_msgs --symlink-install
```

### Python Import Errors
```bash
# Solution: Make sure you're using correct Python virtual environment
python3 --version  # Should be 3.10+

# Check ROS 2 environment
ros2 --version  # Should be Humble (12.x)
```

### "ModuleNotFoundError" or Import Fails
```bash
# Solution: Ensure all dependencies installed
pip list | grep ros  # Should see ROS 2 packages

# Reinstall ROS 2 setup tools if needed
pip install setuptools wheel
```

---

## 📊 BUILD TIME EXPECTATIONS

| Step | Time | Status |
|------|------|--------|
| Step 1: Prepare system | 5 min | Quick |
| Step 2: Create workspace | 5 min | Quick |
| Step 3: Clone packages | 10 min | Network dependent |
| Step 4: Copy files | 30 min | Manual |
| Step 5: Build rover_msgs | 5 min | Quick |
| Step 6: Build source packages | 15 min | Slow (C++ compilation) |
| Step 7: Build custom packages | 20 min | Slow (C++ compilation) |
| Step 8: Source workspace | 1 min | Quick |
| Step 9: Verify build | 5 min | Quick |
| Step 10: Test packages | 15 min | Quick |
| **TOTAL** | **~1.5-2 hours** | Typical |

---

**Build complete? → Start with 06_LAUNCH_FILES.md to test phases!**
