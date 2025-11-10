cmake_minimum_required(VERSION 3.8)
project(somanet)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(joy REQUIRED)

# Dependencies for certified parameter validation (ISO 13849-1 §5.2.2)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENSSL REQUIRED IMPORTED_TARGET OpenSSL::SSL OpenSSL::Crypto)
pkg_check_modules(YAML_CPP REQUIRED IMPORTED_TARGET yaml-cpp)

# Try to find SOEM (EtherCAT library)
# Note: SOEM may not have a proper CMake config, so we try multiple approaches
find_path(SOEM_INCLUDE_DIR
  NAMES soem/ethercattype.h
  PATHS
    /usr/local/include
    /usr/include
    /opt/ros/${ROS_DISTRO}/include
  PATH_SUFFIXES soem
)

find_library(SOEM_LIBRARY
  NAMES soem
  PATHS
    /usr/local/lib
    /usr/lib
    /opt/ros/${ROS_DISTRO}/lib
)

if(SOEM_INCLUDE_DIR AND SOEM_LIBRARY)
  message(STATUS "Found SOEM: ${SOEM_LIBRARY}")
  message(STATUS "SOEM include dir: ${SOEM_INCLUDE_DIR}")
  add_compile_definitions(HAVE_SOEM)
else()
  message(WARNING "SOEM not found - EtherCAT functionality may not work")
  message(WARNING "Install with: sudo apt-get install soem or check your installation path")
endif()

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SOEM_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Build modular library components
add_library(ethercat_driver_lib
  src/ethercat_driver.cpp
)

# Link SOEM library if found
if(SOEM_LIBRARY)
  target_link_libraries(ethercat_driver_lib
    ${SOEM_LIBRARY}
    pthread
  )
else()
  target_link_libraries(ethercat_driver_lib
    pthread
  )
endif()

# Add ROS2 dependencies to ethercat_driver_lib
ament_target_dependencies(ethercat_driver_lib
  rclcpp
  std_msgs
  geometry_msgs
)

add_library(odometry_calculator_lib
  src/odometry_calculator.cpp
)

# Add ROS2 dependencies to odometry_calculator_lib
ament_target_dependencies(odometry_calculator_lib
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
)

# Build main executable (legacy monolithic version)
set(SOURCES_LEGACY src/main.cpp)
add_library(LibsModule ${SOURCES_LEGACY})

# Link libraries properly
if(SOEM_LIBRARY)
  target_link_libraries(LibsModule
    ${SOEM_LIBRARY}
    pthread
  )
else()
  target_link_libraries(LibsModule
    pthread
  )
endif()

ament_target_dependencies(LibsModule
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  ament_index_cpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(main ${SOURCES_LEGACY})
target_link_libraries(main 
  LibsModule 
  odometry_calculator_lib
  ethercat_driver_lib
)

# Build teleop_joy executable
add_executable(teleop_joy src/teleop_joy.cpp)
ament_target_dependencies(teleop_joy
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

# Build safety_supervisor_node executable
add_executable(safety_supervisor_node 
  src/safety_supervisor_node.cpp
  src/certified_params_validator.cpp
  src/maintenance_mode_manager.cpp
)
ament_target_dependencies(safety_supervisor_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  ament_index_cpp
  geometry_msgs
  nav_msgs
  std_msgs
  diagnostic_msgs
)
target_link_libraries(safety_supervisor_node
  OpenSSL::SSL
  OpenSSL::Crypto
  yaml-cpp
)

# Build command_arbitrator_node executable
add_executable(command_arbitrator_node src/command_arbitrator_node.cpp)
ament_target_dependencies(command_arbitrator_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  std_msgs
  diagnostic_msgs
)

# ============================================================================
# NAV2 INTEGRATION: Command Multiplexer for Autonomous Navigation
# ============================================================================
# Consolidates Nav2 + other autonomous command sources before arbitration
add_executable(command_mux_node src/command_mux_node.cpp)
ament_target_dependencies(command_mux_node
  rclcpp
  geometry_msgs
  std_msgs
  diagnostic_msgs
)

# Install executables
install(TARGETS
  main
  teleop_joy
  safety_supervisor_node
  command_arbitrator_node
  command_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/teleop_keyboard.py
  scripts/teleop_keyboard_safe.py
  scripts/validate_odometry.py
  scripts/generate_certification_hash.py
  scripts/save_map.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install shell scripts (maintenance and verification)
install(PROGRAMS
  scripts/install_dependencies.sh
  scripts/verify_safety_integration.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install PowerShell scripts (Windows compatibility)
install(FILES
  scripts/verify_safety_integration.ps1
  scripts/verify_velocity_limits.ps1
  DESTINATION lib/${PROJECT_NAME}
)

# Install public headers (for other packages to use)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# Install config files (including behavior trees)
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF/robot description files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "*.urdf" PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
  PATTERN "*.xacro" PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
)

# Install documentation
install(FILES
  README.md
  README_NAV2.md
  SAFETY.md
  BUILD_GUIDE.md
  SROS2_GUIDE.md
  SLAM_GUIDE.md
  BEHAVIOR_TREE_GUIDE.md
  CALIBRATION_GUIDE.md
  CHANGELOG.md
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(launch_testing REQUIRED)
  
  # Unit test: Odometry Calculator
  ament_add_gtest(test_odometry_calculator 
    test/test_odometry_calculator.cpp
  )
  target_link_libraries(test_odometry_calculator
    odometry_calculator_lib
  )
  ament_target_dependencies(test_odometry_calculator
    rclcpp
    geometry_msgs
    nav_msgs
  )
  
  # Unit test: Mock Driver
  ament_add_gtest(test_mock_driver 
    test/test_mock_driver.cpp
  )
  target_include_directories(test_mock_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_mock_driver
    rclcpp
  )
  
  # Integration test: Safety-Critical Behavior (ISO 13849-1)
  ament_add_gtest(test_safety_critical 
    test/test_safety_critical.cpp
  )
  ament_target_dependencies(test_safety_critical
    rclcpp
    geometry_msgs
    std_msgs
    nav_msgs
  )
  target_link_libraries(test_safety_critical
    odometry_calculator_lib
  )
  
  # Unit test: Command Arbitrator
  ament_add_gtest(test_command_arbitrator 
    test/test_command_arbitrator.cpp
  )
  ament_target_dependencies(test_command_arbitrator
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    geometry_msgs
    std_msgs
    diagnostic_msgs
  )
  
  # Unit test: Safety Supervisor
  ament_add_gtest(test_safety_supervisor 
    test/test_safety_supervisor.cpp
  )
  ament_target_dependencies(test_safety_supervisor
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    diagnostic_msgs
  )
  
  # Unit test: Command Multiplexer
  ament_add_gtest(test_command_mux 
    test/test_command_mux.cpp
  )
  ament_target_dependencies(test_command_mux
    rclcpp
    geometry_msgs
    std_msgs
    diagnostic_msgs
  )
  target_link_libraries(test_command_mux command_mux_node)
  
  # Integration test: EtherCAT Failure Detection
  ament_add_gtest(test_ethercat_failures 
    test/test_ethercat_failures.cpp
  )
  target_include_directories(test_ethercat_failures PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_ethercat_failures
    rclcpp
  )

  # Integration test: Lifecycle Node Bring-up
  add_executable(test_lifecycle_integration
    test/test_lifecycle_integration.cpp
  )
  ament_target_dependencies(test_lifecycle_integration
    rclcpp
    launch_testing
    lifecycle_msgs
    ament_index_cpp
  )
  install(TARGETS
    test_lifecycle_integration
    DESTINATION lib/${PROJECT_NAME}
  )
  
  # Lint tests
  ament_lint_auto_find_test_dependencies()
  
  # Install test executables for manual execution
  install(TARGETS
    test_odometry_calculator
    test_mock_driver
    test_safety_critical
    test_command_arbitrator
    test_command_mux
    test_safety_supervisor
    test_ethercat_failures
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()


