# PHASE 2: STATIC TF PUBLISHER NODE
# Place this in: src/static_tf_publisher/

## FILE STRUCTURE:
```
static_tf_publisher/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── static_tf_publisher.cpp
├── config/
│   ├── static_transforms.yaml
│   └── rover_robot.urdf
└── launch/
    └── static_transforms.launch.py
```

---

## FILE 1: static_tf_publisher/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>static_tf_publisher</name>
  <version>0.0.1</version>
  <description>Static TF broadcaster for rover frame tree</description>
  <maintainer email="rover@moonbase.local">Rover Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rclcpp</build_depend>
  <build_depend>tf2</build_depend>
  <build_depend>tf2_ros</build_depend>
  <build_depend>geometry_msgs</build_depend>

  <exec_depend>rclcpp</exec_depend>
  <exec_depend>tf2</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## FILE 2: static_tf_publisher/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(static_tf_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(static_tf_broadcaster
  src/static_tf_publisher.cpp
)

ament_target_dependencies(static_tf_broadcaster
  rclcpp
  tf2_ros
  geometry_msgs
)

install(TARGETS static_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

---

## FILE 3: static_tf_publisher/src/static_tf_publisher.cpp

```cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

/**
 * Static transform publisher for Moon Rover TF tree:
 * 
 * map
 *  └─ odom (published by EKF)
 *      └─ base_link (published by EKF)
 *          ├─ camera_link (published here)
 *          ├─ tof_link (published here)
 *          └─ imu_link (published here)
 */
class StaticTFPublisher : public rclcpp::Node {
public:
  StaticTFPublisher()
    : rclcpp::Node("static_tf_publisher")
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // base_link -> camera_link
    // Camera mounted horizontally on rover front
    publishStaticTransform(
      "base_link", "camera_link",
      0.20,   // x offset: 20 cm forward
      0.0,    // y offset: center
      0.10,   // z offset: 10 cm up
      0.0, 0.0, 0.0, 1.0);  // No rotation (camera axis aligned with rover)

    // base_link -> tof_link
    // ToF sensor mounted next to RGB camera
    publishStaticTransform(
      "base_link", "tof_link",
      0.20,   // x offset: 20 cm forward (same as camera)
      0.05,   // y offset: 5 cm right
      0.10,   // z offset: 10 cm up
      0.0, 0.0, 0.0, 1.0);  // Same orientation as camera

    // base_link -> imu_link
    // IMU at center of rover
    publishStaticTransform(
      "base_link", "imu_link",
      0.0,    // x offset: center
      0.0,    // y offset: center
      0.05,   // z offset: 5 cm up
      0.0, 0.0, 0.0, 1.0);  // No rotation

    // base_link -> vex_base (optional, for motor control reference)
    publishStaticTransform(
      "base_link", "vex_base",
      0.0,    // x offset: center
      0.0,    // y offset: center
      0.0,    // z offset: same
      0.0, 0.0, 0.0, 1.0);

    RCLCPP_INFO(get_logger(), "Static TF tree published successfully");
  }

private:
  void publishStaticTransform(
    const std::string& parent_frame,
    const std::string& child_frame,
    double x, double y, double z,
    double qx, double qy, double qz, double qw)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(0);
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;

    // Translation
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;

    // Rotation (quaternion)
    transform.transform.rotation.x = qx;
    transform.transform.rotation.y = qy;
    transform.transform.rotation.z = qz;
    transform.transform.rotation.w = qw;

    tf_static_broadcaster_->sendTransform(transform);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticTFPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

---

## FILE 4: static_tf_publisher/config/static_transforms.yaml

```yaml
# Static TF configuration for Moon Rover
# All offsets in meters, relative to base_link

transforms:
  base_link_to_camera:
    parent: "base_link"
    child: "camera_link"
    position:
      x: 0.20        # 20 cm forward
      y: 0.0         # center
      z: 0.10        # 10 cm up
    rotation:  # identity (no rotation)
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0

  base_link_to_tof:
    parent: "base_link"
    child: "tof_link"
    position:
      x: 0.20        # Same forward as camera
      y: 0.05        # 5 cm right offset
      z: 0.10        # Same height
    rotation:  # identity
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0

  base_link_to_imu:
    parent: "base_link"
    child: "imu_link"
    position:
      x: 0.0         # center
      y: 0.0         # center
      z: 0.05        # 5 cm up
    rotation:  # identity
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

---

## FILE 5: static_tf_publisher/config/rover_robot.urdf

```xml
<?xml version="1.0"?>
<robot name="moon_rover">
  <!-- Base link (center of rotation) -->
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.2"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.3 0.2 0.15"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Camera link (RGB IMX500) -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.04 0.04 0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Joint: base_link -> camera_link -->
  <joint name="base_link_to_camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.20 0.0 0.10" rpy="0 0 0"/>
  </joint>

  <!-- ToF link (depth camera) -->
  <link name="tof_link">
    <visual>
      <geometry>
        <box size="0.03 0.03 0.03"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- Joint: base_link -> tof_link -->
  <joint name="base_link_to_tof_joint" type="fixed">
    <parent link="base_link"/>
    <child link="tof_link"/>
    <origin xyz="0.20 0.05 0.10" rpy="0 0 0"/>
  </joint>

  <!-- IMU link -->
  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.1 0.1 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Joint: base_link -> imu_link -->
  <joint name="base_link_to_imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
  </joint>

</robot>
```

---

## FILE 6: static_tf_publisher/launch/static_transforms.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='static_tf_publisher',
            executable='static_tf_broadcaster',
            name='static_tf_publisher',
            output='screen'
        )
    ])
```

---

## BUILD & TEST

```bash
cd ~/ros2_moon_rover

# Build static TF publisher
colcon build --packages-select static_tf_publisher --symlink-install

# Test
source install/setup.bash
ros2 run static_tf_publisher static_tf_broadcaster

# In another terminal, verify TF tree:
ros2 run tf2_tools view_frames.py
evince frames.pdf

# Expected tree:
# base_link
# ├─ camera_link
# ├─ tof_link
# └─ imu_link
```

---

# CRITICAL: TF CALIBRATION

The static transforms above are **ESTIMATES**. You MUST calibrate:

1. **Measure on rover**: Use ruler/calipers to get exact:
   - Distance from base_link center to camera lens
   - Distance from base_link center to ToF sensor
   - Distance from base_link center to IMU chip

2. **Update YAML/URDF** with actual measurements

3. **Verify with RViz**: Load rover_robot.urdf and check sensor placement visually

4. **Cross-check with vision**: Project known landmarks with different TF offsets, verify in RViz

---

## NEXT: EKF Fusion Node

Request **06_EKF_FUSION_NODE.md**
