
# Terrain Mapping Rover Documentation

## Function

### Primary Functions:
1. **Visual SLAM** — Builds 3D map using camera + depth sensor + wheel odometry
2. **State Estimation** — Fuses wheel encoders + IMU with EKF filter
3. **Autonomous Navigation** — Plans and follows waypoints using Nav2
4. **Terrain Perception** — Classifies terrain and detects landmarks
5. **Mission Monitoring** — Tracks health, logs data, streams video

### Operating Environment:
- **Platform:** Raspberry Pi 5 (8+GB RAM)
- **Terrain:** Simulated moon (low-texture, harsh lighting, uneven ground)
- **Sensors:** 
  - Pi AI camera (IMX500)
  - Arducam ToF depth camera
  - IMU (MPU6050)
  - Wheel encoders (via VEX V5 brain)
- **Duration:** TBD

## PREREQUISITES

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
source /ros2_humble/install/setup.bash
echo $ROS_DISTRO
# should show humble

# Check colcon
colcon version-check

# Check git
git --version

# If any of these fail, install missing packages:
# ROS 2: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
# colcon: sudo apt install python3-colcon-common-extensions
# git: sudo apt install git
```

## 🚀 STEP 0A: Prepare V5 Brain & Install PROS CLI

### Prerequisites
- VEX V5 Robot Brain (connected via USB)
- Computer with Python 3.7+
- Internet connection

### Installation

```bash
# 1. Install PROS CLI
pipx install pros-cli

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
lsusb
# Should show: Bus 001 Device 003: ID 2888:0501 VEX Robotics, Inc VEX Robotics V5 Brain - 155E4C00

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

## 🔧 STEP 0B: Create PROS Project & Add rosserial

### Create New Project

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


