# PHASE 1: VEX SERIAL DRIVER NODE
# Place this in: src/vex_driver_node/

## FILE STRUCTURE:
```
vex_driver_node/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── vex_driver_node.cpp
├── include/vex_driver_node/
│   └── vex_driver.hpp
└── config/
    └── vex_params.yaml
```

---

## FILE 1: vex_driver_node/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>vex_driver_node</name>
  <version>0.0.1</version>
  <description>VEX V5 serial driver for wheel odometry</description>
  <maintainer email="rover@moonbase.local">Rover Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <build_depend>rclcpp</build_depend>
  <build_depend>nav_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>tf2</build_depend>
  <build_depend>tf2_ros</build_depend>

  <exec_depend>rclcpp</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>tf2</exec_depend>
  <exec_depend>tf2_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## FILE 2: vex_driver_node/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(vex_driver_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(vex_driver_node
  src/vex_driver_node.cpp
)

ament_target_dependencies(vex_driver_node
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

target_include_directories(vex_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS vex_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

---

## FILE 3: vex_driver_node/include/vex_driver_node/vex_driver.hpp

```cpp
#ifndef VEX_DRIVER_HPP_
#define VEX_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <cmath>

class VexDriverNode : public rclcpp::Node {
public:
  VexDriverNode();
  ~VexDriverNode();

private:
  // Serial communication
  int serial_fd_;
  bool setupSerialPort(const std::string& port, int baudrate);
  bool readFromSerial(uint8_t* buffer, size_t len);
  bool writeToSerial(const uint8_t* data, size_t len);

  // Odometry calculation
  void timerCallback();
  void parseVexMessage(const std::string& message);
  
  // ROS 2 components
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Odometry state
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
  double vx_ = 0.0, vtheta_ = 0.0;
  rclcpp::Time last_time_;

  // Configuration
  std::string serial_port_;
  int baud_rate_;
  double wheel_radius_;       // meters
  double wheel_separation_;   // meters
  double encoder_counts_per_rev_;
  bool enable_tf_broadcast_;

  // Raw encoder values
  int32_t left_encoder_counts_ = 0, right_encoder_counts_ = 0;
  int32_t left_encoder_prev_ = 0, right_encoder_prev_ = 0;

  // Covariance matrix (6x6)
  std::array<double, 36> pose_covariance_;
  std::array<double, 36> twist_covariance_;
};

#endif  // VEX_DRIVER_HPP_
```

---

## FILE 4: vex_driver_node/src/vex_driver_node.cpp

```cpp
#include "vex_driver_node/vex_driver.hpp"
#include <iostream>
#include <sstream>
#include <cstring>

VexDriverNode::VexDriverNode()
  : rclcpp::Node("vex_driver_node"),
    serial_fd_(-1),
    last_time_(now())
{
  // Declare parameters
  declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  declare_parameter<int>("baud_rate", 115200);
  declare_parameter<double>("wheel_radius", 0.05);  // 5 cm wheels
  declare_parameter<double>("wheel_separation", 0.15);  // 15 cm track width
  declare_parameter<double>("encoder_counts_per_rev", 1440.0);  // VEX encoder
  declare_parameter<bool>("enable_tf_broadcast", true);

  // Get parameters
  serial_port_ = get_parameter("serial_port").as_string();
  baud_rate_ = get_parameter("baud_rate").as_int();
  wheel_radius_ = get_parameter("wheel_radius").as_double();
  wheel_separation_ = get_parameter("wheel_separation").as_double();
  encoder_counts_per_rev_ = get_parameter("encoder_counts_per_rev").as_double();
  enable_tf_broadcast_ = get_parameter("enable_tf_broadcast").as_bool();

  // Initialize covariance matrices
  // Odometry from wheel encoders - relatively low uncertainty
  pose_covariance_.fill(0.0);
  pose_covariance_[0] = 0.01;     // x variance
  pose_covariance_[7] = 0.01;     // y variance
  pose_covariance_[14] = 0.05;    // theta variance (radians^2)

  twist_covariance_.fill(0.0);
  twist_covariance_[0] = 0.005;   // vx variance
  twist_covariance_[35] = 0.01;   // vtheta variance

  // Setup serial port
  if (!setupSerialPort(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Opened serial port: %s @ %d baud", serial_port_.c_str(), baud_rate_);

  // Create publishers
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/vex/odom_raw", 10);
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/vex/joint_state", 10);

  // TF broadcaster
  if (enable_tf_broadcast_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

  // Timer callback at 50 Hz (20 ms)
  timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&VexDriverNode::timerCallback, this)
  );

  RCLCPP_INFO(get_logger(), "VEX driver initialized, publishing at 50 Hz");
}

VexDriverNode::~VexDriverNode() {
  if (serial_fd_ >= 0) {
    close(serial_fd_);
  }
}

bool VexDriverNode::setupSerialPort(const std::string& port, int baudrate) {
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    close(serial_fd_);
    return false;
  }

  // Set baud rate
  speed_t speed;
  switch (baudrate) {
    case 115200: speed = B115200; break;
    case 57600:  speed = B57600; break;
    case 38400:  speed = B38400; break;
    case 19200:  speed = B19200; break;
    case 9600:   speed = B9600; break;
    default:     speed = B115200;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;

  // No flow control
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  // Raw input
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_iflag = 0;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    close(serial_fd_);
    return false;
  }

  return true;
}

bool VexDriverNode::readFromSerial(uint8_t* buffer, size_t len) {
  ssize_t n = read(serial_fd_, buffer, len);
  return n > 0;
}

bool VexDriverNode::writeToSerial(const uint8_t* data, size_t len) {
  return write(serial_fd_, data, len) > 0;
}

void VexDriverNode::timerCallback() {
  // Read serial data and parse odometry messages
  // Expected message format: "ENC:L=<left_counts>,R=<right_counts>\n"
  
  uint8_t buffer[256];
  static std::string serial_buffer = "";

  if (readFromSerial(buffer, 255)) {
    buffer[255] = '\0';
    serial_buffer += std::string(reinterpret_cast<char*>(buffer));

    // Parse complete messages
    size_t pos = 0;
    while ((pos = serial_buffer.find('\n')) != std::string::npos) {
      std::string message = serial_buffer.substr(0, pos);
      serial_buffer.erase(0, pos + 1);

      if (!message.empty()) {
        parseVexMessage(message);
      }
    }
  }

  // Publish odometry at 50 Hz regardless
  rclcpp::Time current_time = now();
  double dt = (current_time - last_time_).seconds();
  if (dt <= 0.0) dt = 0.02;  // Default to 50 Hz if time goes backward

  // Calculate distances from encoder deltas
  double left_distance = (left_encoder_counts_ - left_encoder_prev_) 
                         / encoder_counts_per_rev_ * (2 * M_PI * wheel_radius_);
  double right_distance = (right_encoder_counts_ - right_encoder_prev_) 
                          / encoder_counts_per_rev_ * (2 * M_PI * wheel_radius_);

  left_encoder_prev_ = left_encoder_counts_;
  right_encoder_prev_ = right_encoder_counts_;

  // Differential drive kinematics
  double distance = (left_distance + right_distance) / 2.0;
  double dtheta = (right_distance - left_distance) / wheel_separation_;

  // Update pose
  x_ += distance * std::cos(theta_ + dtheta / 2.0);
  y_ += distance * std::sin(theta_ + dtheta / 2.0);
  theta_ += dtheta;

  // Velocities
  vx_ = distance / dt;
  vtheta_ = dtheta / dt;

  // Normalize theta
  while (theta_ > M_PI) theta_ -= 2 * M_PI;
  while (theta_ < -M_PI) theta_ += 2 * M_PI;

  // Publish odometry
  auto odom = nav_msgs::msg::Odometry();
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;

  // Convert theta to quaternion
  double cy = std::cos(theta_ / 2.0);
  double sy = std::sin(theta_ / 2.0);
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sy;
  odom.pose.pose.orientation.w = cy;

  // Pose covariance
  for (size_t i = 0; i < 36; ++i) {
    odom.pose.covariance[i] = pose_covariance_[i];
  }

  // Velocity
  odom.twist.twist.linear.x = vx_;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = vtheta_;

  for (size_t i = 0; i < 36; ++i) {
    odom.twist.covariance[i] = twist_covariance_[i];
  }

  odom_pub_->publish(odom);

  // Publish TF if enabled
  if (enable_tf_broadcast_) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = sy;
    t.transform.rotation.w = cy;
    tf_broadcaster_->sendTransform(t);
  }

  // Also publish joint states for wheel angles
  auto joint_state = sensor_msgs::msg::JointState();
  joint_state.header.stamp = current_time;
  joint_state.name = {"left_wheel", "right_wheel"};
  double left_angle = left_encoder_counts_ / encoder_counts_per_rev_ * 2 * M_PI;
  double right_angle = right_encoder_counts_ / encoder_counts_per_rev_ * 2 * M_PI;
  joint_state.position = {left_angle, right_angle};
  joint_state.velocity = {left_distance / dt, right_distance / dt};
  joint_state_pub_->publish(joint_state);

  last_time_ = current_time;
}

void VexDriverNode::parseVexMessage(const std::string& message) {
  // Parse: "ENC:L=12345,R=67890"
  if (message.find("ENC:") == 0) {
    size_t l_pos = message.find("L=");
    size_t comma_pos = message.find(",");
    size_t r_pos = message.find("R=");

    if (l_pos != std::string::npos && comma_pos != std::string::npos && r_pos != std::string::npos) {
      try {
        left_encoder_counts_ = std::stoi(message.substr(l_pos + 2, comma_pos - l_pos - 2));
        right_encoder_counts_ = std::stoi(message.substr(r_pos + 2));
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Failed to parse encoder message: %s", message.c_str());
      }
    }
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VexDriverNode>());
  rclcpp::shutdown();
  return 0;
}
```

---

## FILE 5: vex_driver_node/config/vex_params.yaml

```yaml
vex_driver_node:
  ros__parameters:
    # Serial port configuration
    serial_port: "/dev/ttyUSB0"      # VEX V5 serial port
    baud_rate: 115200                 # Communication speed

    # Hardware configuration
    wheel_radius: 0.05                # 5 cm wheel radius (meters)
    wheel_separation: 0.15            # 15 cm distance between wheels (meters)
    encoder_counts_per_rev: 1440.0    # VEX encoder: 36 counts * 40:1 gearing

    # TF publishing
    enable_tf_broadcast: true         # Publish odom -> base_link transform

    # Tuning notes:
    # - Increase encoder_counts_per_rev if odometry drifts backward
    # - Increase wheel_separation if yaw drifts during straight lines
    # - Adjust wheel_radius if linear distance is consistently wrong
    #   (e.g., if rover travels 1 m but reports 1.1 m, decrease radius by ~9%)
```

---

## BUILD & TEST

```bash
cd ~/ros2_moon_rover

# Build VEX driver
colcon build --packages-select vex_driver_node --symlink-install

# Test (requires serial connection to VEX V5)
source install/setup.bash
ros2 run vex_driver_node vex_driver_node --ros-args --params-file src/vex_driver_node/config/vex_params.yaml

# In another terminal, monitor:
ros2 topic echo /vex/odom_raw
```
