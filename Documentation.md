
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


