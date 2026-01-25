cmake_minimum_required(VERSION 3.8)
project(tmr_costmap)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
find_package(pcl_conversions REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# =============================================================================
# Terrain Classifier Library
# =============================================================================
add_library(terrain_classifier SHARED
  src/terrain_classifier.cpp
  src/point_cloud_processor.cpp
)
target_include_directories(terrain_classifier PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(terrain_classifier
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)
target_link_libraries(terrain_classifier ${PCL_LIBRARIES})

# =============================================================================
# ToF Terrain Layer Plugin
# =============================================================================
add_library(tof_terrain_layer SHARED
  src/tof_terrain_layer.cpp
)
target_include_directories(tof_terrain_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tof_terrain_layer
  rclcpp
  nav2_costmap_2d
  nav2_core
  nav2_util
  pluginlib
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  message_filters
)
target_link_libraries(tof_terrain_layer
  terrain_classifier
  ${PCL_LIBRARIES}
)

# =============================================================================
# Install
# =============================================================================

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS terrain_classifier tof_terrain_layer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin description
install(FILES plugins/costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install rviz config
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install Python scripts
install(PROGRAMS
  scripts/terrain_visualizer.py
  scripts/costmap_diagnostics.py
  scripts/test_terrain_layer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install docs
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

# =============================================================================
# Export
# =============================================================================

# Export plugin
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins/costmap_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(terrain_classifier tof_terrain_layer)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  nav2_costmap_2d
  nav2_core
  pluginlib
  sensor_msgs
  tf2_ros
)

# =============================================================================
# Testing
# =============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  # Unit tests
  ament_add_gtest(test_terrain_classifier
    test/test_terrain_classifier.cpp
  )
  target_link_libraries(test_terrain_classifier terrain_classifier)
  ament_target_dependencies(test_terrain_classifier
    rclcpp
    sensor_msgs
  )
endif()

ament_package()
