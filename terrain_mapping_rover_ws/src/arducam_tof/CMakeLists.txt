cmake_minimum_required(VERSION 3.8)
project(arducam_tof)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/arducam_tof_node.py
  scripts/tof_mock_node.py
  scripts/depth_to_pointcloud_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
