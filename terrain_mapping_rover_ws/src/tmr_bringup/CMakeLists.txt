cmake_minimum_required(VERSION 3.8)
project(tmr_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install all scripts
install(PROGRAMS
  scripts/phase1_system_check.py
  scripts/phase1_motor_test.py
  scripts/phase1_sensor_test.py
  scripts/phase1_integration_test.py
  scripts/phase1_calibration.py
  scripts/phase1_quick_test.py
  scripts/phase1_record_baseline.py
  scripts/phase1_health_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install test files
install(DIRECTORY test/
  DESTINATION share/${PROJECT_NAME}/test
)

# Install documentation
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
