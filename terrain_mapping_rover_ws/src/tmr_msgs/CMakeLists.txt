cmake_minimum_required(VERSION 3.8)
project(tmr_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Message files
set(msg_files
  "msg/Classification.msg"
  "msg/ClassificationArray.msg"
  "msg/LandmarkDetection.msg"
  "msg/LandmarkDetectionArray.msg"
  "msg/LandmarkPosition.msg"
  "msg/LandmarkPositionArray.msg"
  "msg/LandmarkVerification.msg"
  "msg/LandmarkMap.msg"
  "msg/MissionHealth.msg"
  "msg/MissionEvent.msg"
  "msg/WaypointList.msg"
  "msg/BluetoothBeacon.msg"
  "msg/CrossRoverSyncStatus.msg"
  "msg/VexStatus.msg"
  "msg/TankDriveCmd.msg"
)

# Service files
set(srv_files
  "srv/RecordWaypoint.srv"
  "srv/SetTargetLocation.srv"
  "srv/StartBluetoothNav.srv"
  "srv/EmergencyStop.srv"
  "srv/ClearEmergencyStop.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
