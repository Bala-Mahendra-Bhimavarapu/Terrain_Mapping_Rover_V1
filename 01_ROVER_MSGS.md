# rover_msgs - Custom Message Definitions
# Place this entire section in: src/rover_msgs/

## FILE STRUCTURE:
```
rover_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── Odometry2D.msg
    ├── Classification.msg
    ├── LandmarkDetection.msg
    ├── Landmark3D.msg
    ├── LandmarkMap.msg
    ├── MissionHealth.msg
    ├── MissionEvent.msg
    └── LandmarkVerification.msg
```

---

## FILE 1: rover_msgs/package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rover_msgs</name>
  <version>0.0.1</version>
  <description>Custom ROS 2 message definitions for Moon Rover SLAM system</description>
  <maintainer email="rover@moonbase.local">Rover Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_cmake</buildtool_depend>

  <build_depend>builtin_interfaces</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>std_msgs</build_depend>

  <exec_depend>builtin_interfaces</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>rosidl_runtime_cpp</exec_depend>
  <exec_depend>rosidl_runtime_py</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## FILE 2: rover_msgs/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(rover_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/Odometry2D.msg"
  "msg/Classification.msg"
  "msg/LandmarkDetection.msg"
  "msg/Landmark3D.msg"
  "msg/LandmarkMap.msg"
  "msg/MissionHealth.msg"
  "msg/MissionEvent.msg"
  "msg/LandmarkVerification.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    sensor_msgs
    std_msgs
)

ament_package()
```

---

## FILE 3: rover_msgs/msg/Odometry2D.msg

```
# 2D Odometry with covariance (wheel encoders, dead reckoning)
std_msgs/Header header

# Position (x, y in odom frame)
float64 x
float64 y

# Orientation (yaw in radians)
float64 yaw

# Velocities
float64 vx        # Linear velocity (m/s)
float64 vtheta    # Angular velocity (rad/s)

# Covariance matrix (6x6 flattened)
# Order: x, y, theta, vx, vy, vtheta
float64[36] pose_covariance
```

---

## FILE 4: rover_msgs/msg/Classification.msg

```
# AI camera classification result
std_msgs/Header header

string label           # Object class name (e.g., "rock", "rover", "crater")
float32 confidence     # Confidence [0.0, 1.0]

# Bounding box in image
uint32 bbox_x_min
uint32 bbox_y_min
uint32 bbox_x_max
uint32 bbox_y_max

# Optional: index in image
uint32 detection_id
```

---

## FILE 5: rover_msgs/msg/LandmarkDetection.msg

```
# YOLO landmark detection in image space
std_msgs/Header header

string label              # Landmark class
float32 confidence        # [0.0, 1.0]

# Bounding box
uint32 bbox_x_min
uint32 bbox_y_min
uint32 bbox_x_max
uint32 bbox_y_max

# Centroid in image
float32 center_x          # pixels
float32 center_y          # pixels

uint32 detection_id       # Unique ID for tracking
```

---

## FILE 6: rover_msgs/msg/Landmark3D.msg

```
# 3D Landmark position (in map frame)
std_msgs/Header header

string label              # Landmark name
geometry_msgs/Point position   # x, y, z in map frame

# Uncertainty
geometry_msgs/Point covariance_diag  # Diagonal of 3x3 covariance (sigma_x, sigma_y, sigma_z)

float32 confidence        # [0.0, 1.0]
uint32 detection_count    # Times observed

builtin_interfaces/Time last_observation
```

---

## FILE 7: rover_msgs/msg/LandmarkMap.msg

```
# Complete map of landmarks (persistent)
std_msgs/Header header

Landmark3D[] landmarks     # Array of all known landmarks
uint32 total_landmarks
builtin_interfaces/Time last_updated

# Statistics
uint32 verified_count      # Landmarks verified cross-rover
uint32 unverified_count
```

---

## FILE 8: rover_msgs/msg/MissionHealth.msg

```
# Mission health status
std_msgs/Header header

# Status levels: OK, DEGRADED, CRITICAL
uint8 status
uint8 STATUS_OK = 0
uint8 STATUS_DEGRADED = 1
uint8 STATUS_CRITICAL = 2

# Component health
uint8 slam_health
uint8 odometry_health
uint8 perception_health
uint8 navigation_health
uint8 landmark_health

# Numeric indicators
float32 loop_closure_rate      # [0.0, 1.0] - fraction of poses with loop closure
uint32 feature_count           # Active features in current frame
float32 reprojection_error      # RMS error (pixels)
float32 odom_covariance_trace   # Sum of diagonal covariance

string[] warning_messages       # Human-readable warnings
```

---

## FILE 9: rover_msgs/msg/MissionEvent.msg

```
# Mission event log entry
std_msgs/Header header

uint8 severity
uint8 SEV_INFO = 0
uint8 SEV_WARNING = 1
uint8 SEV_ERROR = 2
uint8 SEV_CRITICAL = 3

string component      # Which node generated event (e.g., "rtabmap", "ekf", "nav2")
string event_type     # Type: "loop_closure", "landmark_detected", "goal_reached", "obstacle"
string message        # Human-readable message
```

---

## FILE 10: rover_msgs/msg/LandmarkVerification.msg

```
# Cross-rover landmark verification result
std_msgs/Header header

string landmark_label

# Expected position (from map server)
geometry_msgs/Point expected_position

# Observed position (from 3D localizer)
geometry_msgs/Point observed_position

# Error
geometry_msgs/Point position_error   # Difference vector
float32 error_magnitude              # sqrt(dx^2 + dy^2 + dz^2)

# Status
uint8 verification_status
uint8 STATUS_OK = 0          # < 0.3 m error
uint8 STATUS_DEGRADED = 1    # 0.3-1.0 m error
uint8 STATUS_CRITICAL = 2    # > 1.0 m error

builtin_interfaces/Time timestamp
```

---

## BUILD INSTRUCTIONS

```bash
cd ~/ros2_moon_rover

# Build messages FIRST (before other packages depend on them)
colcon build --packages-select rover_msgs --symlink-install

# Verify
source install/setup.bash
ros2 msg list | grep rover_msgs

# Expected output:
# rover_msgs/msg/Classification
# rover_msgs/msg/Landmark3D
# ... etc
```
