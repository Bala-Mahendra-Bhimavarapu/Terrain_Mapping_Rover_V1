# PHASE 1: IMU DRIVER NODE (MPU6050)
# Place this in: src/imu_driver_node/

## FILE STRUCTURE:
```
imu_driver_node/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── imu_driver_node.cpp
├── include/imu_driver_node/
│   └── imu_driver.hpp
└── config/
    └── imu_params.yaml
```

---

## FILE 1: imu_driver_node/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>imu_driver_node</name>
  <version>0.0.1</version>
  <description>MPU6050 IMU driver for orientation and acceleration</description>
  <maintainer email="rover@moonbase.local">Rover Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rclcpp</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>std_msgs</build_depend>

  <exec_depend>rclcpp</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## FILE 2: imu_driver_node/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(imu_driver_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(imu_driver_node
  src/imu_driver_node.cpp
)

ament_target_dependencies(imu_driver_node
  rclcpp
  sensor_msgs
  std_msgs
)

target_include_directories(imu_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS imu_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

---

## FILE 3: imu_driver_node/include/imu_driver_node/imu_driver.hpp

```cpp
#ifndef IMU_DRIVER_HPP_
#define IMU_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

// MPU6050 Register Addresses
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define TEMP_OUT_H 0x41
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B

class IMUDriverNode : public rclcpp::Node {
public:
  IMUDriverNode();
  ~IMUDriverNode();

private:
  // I2C communication
  int i2c_fd_;
  bool setupI2C(const std::string& device, uint8_t addr);
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);
  int16_t read16bit(uint8_t reg);

  // Reading and publishing
  void timerCallback();

  // ROS 2 components
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Calibration offsets
  struct {
    double accel_x_offset = 0.0;
    double accel_y_offset = 0.0;
    double accel_z_offset = -9.81;  // Gravity offset
    double gyro_x_offset = 0.0;
    double gyro_y_offset = 0.0;
    double gyro_z_offset = 0.0;
  } calibration_;

  // Scale factors (from datasheet)
  double accel_scale_ = 9.81 / 16384.0;  // ±2g range
  double gyro_scale_ = 1.0 / 131.0;      // ±250 deg/s range

  // Configuration
  std::string i2c_device_;
  int publish_frequency_;
  bool enable_calibration_;
};

#endif  // IMU_DRIVER_HPP_
```

---

## FILE 4: imu_driver_node/src/imu_driver_node.cpp

```cpp
#include "imu_driver_node/imu_driver.hpp"
#include <iostream>
#include <thread>
#include <chrono>

IMUDriverNode::IMUDriverNode()
  : rclcpp::Node("imu_driver_node"),
    i2c_fd_(-1)
{
  declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
  declare_parameter<int>("publish_frequency", 100);  // 100 Hz
  declare_parameter<bool>("enable_calibration", true);

  i2c_device_ = get_parameter("i2c_device").as_string();
  publish_frequency_ = get_parameter("publish_frequency").as_int();
  enable_calibration_ = get_parameter("enable_calibration").as_bool();

  // Setup I2C
  if (!setupI2C(i2c_device_, MPU6050_ADDR)) {
    RCLCPP_ERROR(get_logger(), "Failed to open I2C device: %s", i2c_device_.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "I2C device initialized: %s", i2c_device_.c_str());

  // Wake up MPU6050 (disable sleep mode)
  writeRegister(PWR_MGMT_1, 0x00);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Set accelerometer range to ±2g (default)
  writeRegister(ACCEL_CONFIG, 0x00);

  // Set gyro range to ±250 deg/s (default)
  writeRegister(GYRO_CONFIG, 0x00);

  // Create publisher
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

  // Timer at specified frequency
  int period_ms = 1000 / publish_frequency_;
  timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&IMUDriverNode::timerCallback, this)
  );

  RCLCPP_INFO(get_logger(), "IMU driver initialized at %d Hz", publish_frequency_);
}

IMUDriverNode::~IMUDriverNode() {
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
  }
}

bool IMUDriverNode::setupI2C(const std::string& device, uint8_t addr) {
  i2c_fd_ = open(device.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    return false;
  }

  if (ioctl(i2c_fd_, I2C_SLAVE, addr) < 0) {
    close(i2c_fd_);
    return false;
  }

  return true;
}

uint8_t IMUDriverNode::readRegister(uint8_t reg) {
  return i2c_smbus_read_byte_data(i2c_fd_, reg);
}

void IMUDriverNode::writeRegister(uint8_t reg, uint8_t value) {
  i2c_smbus_write_byte_data(i2c_fd_, reg, value);
}

int16_t IMUDriverNode::read16bit(uint8_t reg) {
  int16_t value = i2c_smbus_read_word_data(i2c_fd_, reg);
  // Swap bytes (little-endian to big-endian)
  return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF);
}

void IMUDriverNode::timerCallback() {
  // Read accelerometer
  int16_t accel_x_raw = read16bit(ACCEL_XOUT_H);
  int16_t accel_y_raw = read16bit(ACCEL_XOUT_H + 2);
  int16_t accel_z_raw = read16bit(ACCEL_XOUT_H + 4);

  // Read gyroscope
  int16_t gyro_x_raw = read16bit(GYRO_XOUT_H);
  int16_t gyro_y_raw = read16bit(GYRO_XOUT_H + 2);
  int16_t gyro_z_raw = read16bit(GYRO_XOUT_H + 4);

  // Convert to physical units
  double accel_x = accel_x_raw * accel_scale_ + calibration_.accel_x_offset;
  double accel_y = accel_y_raw * accel_scale_ + calibration_.accel_y_offset;
  double accel_z = accel_z_raw * accel_scale_ + calibration_.accel_z_offset;

  double gyro_x = gyro_x_raw * gyro_scale_ - calibration_.gyro_x_offset;
  double gyro_y = gyro_y_raw * gyro_scale_ - calibration_.gyro_y_offset;
  double gyro_z = gyro_z_raw * gyro_scale_ - calibration_.gyro_z_offset;

  // Convert gyro from deg/s to rad/s
  gyro_x *= M_PI / 180.0;
  gyro_y *= M_PI / 180.0;
  gyro_z *= M_PI / 180.0;

  // Create IMU message
  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.header.stamp = now();
  imu_msg.header.frame_id = "imu_link";

  // Accelerometer (m/s^2)
  imu_msg.linear_acceleration.x = accel_x;
  imu_msg.linear_acceleration.y = accel_y;
  imu_msg.linear_acceleration.z = accel_z;

  // Gyroscope (rad/s)
  imu_msg.angular_velocity.x = gyro_x;
  imu_msg.angular_velocity.y = gyro_y;
  imu_msg.angular_velocity.z = gyro_z;

  // Covariance matrices (9x9 layout)
  // Row-major: [x, y, z, ...]
  
  // Accelerometer covariance (m^2/s^4)
  std::fill(imu_msg.linear_acceleration_covariance.begin(),
            imu_msg.linear_acceleration_covariance.end(), -1);  // Unknown
  imu_msg.linear_acceleration_covariance[0] = 0.0004;  // ~0.02 m/s^2 std
  imu_msg.linear_acceleration_covariance[4] = 0.0004;
  imu_msg.linear_acceleration_covariance[8] = 0.0004;

  // Gyroscope covariance (rad^2/s^2)
  std::fill(imu_msg.angular_velocity_covariance.begin(),
            imu_msg.angular_velocity_covariance.end(), -1);  // Unknown
  imu_msg.angular_velocity_covariance[0] = 0.0001;  // ~0.01 rad/s std
  imu_msg.angular_velocity_covariance[4] = 0.0001;
  imu_msg.angular_velocity_covariance[8] = 0.0001;

  // Orientation not available from accelerometer/gyro alone (no magnetometer)
  std::fill(imu_msg.orientation_covariance.begin(),
            imu_msg.orientation_covariance.end(), -1);

  imu_pub_->publish(imu_msg);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUDriverNode>());
  rclcpp::shutdown();
  return 0;
}
```

---

## FILE 5: imu_driver_node/config/imu_params.yaml

```yaml
imu_driver_node:
  ros__parameters:
    # I2C Configuration
    i2c_device: "/dev/i2c-1"           # Raspberry Pi I2C bus 1
    publish_frequency: 100              # 100 Hz publishing rate

    # Calibration
    enable_calibration: true            # Apply offset corrections

    # NOTE: Calibration procedure:
    # 1. Place rover on flat, level surface
    # 2. Run: ros2 run imu_driver_node imu_driver_node
    # 3. Collect 500 samples: ros2 topic echo /imu/data --csv > imu_data.csv
    # 4. Calculate mean of acceleration when stationary (should be 0,0,-9.81)
    # 5. Update accel_x_offset, accel_y_offset to negative of mean acceleration
    # 6. Calculate mean gyro when stationary (should be 0,0,0)
    # 7. Update gyro_*_offset to negative of mean
```

---

## BUILD & TEST

```bash
cd ~/ros2_moon_rover

# Build IMU driver
colcon build --packages-select imu_driver_node --symlink-install

# Test (requires I2C MPU6050 at 0x68)
source install/setup.bash
ros2 run imu_driver_node imu_driver_node --ros-args --params-file src/imu_driver_node/config/imu_params.yaml

# Monitor in another terminal:
ros2 topic echo /imu/data

# Expected output (when stationary):
# linear_acceleration.z: -9.81 (gravity)
# angular_velocity: all ~0
```
