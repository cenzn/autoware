cmake_minimum_required(VERSION 3.8)
project(qcar2_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(qcar2_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(autoware_command_convert src/autoware_command_convert.cpp)
ament_target_dependencies(autoware_command_convert rclcpp autoware_control_msgs std_msgs qcar2_interfaces)

add_executable(autoware_control_to_qcar_command src/autoware_control_to_qcar_command.cpp)
ament_target_dependencies(autoware_control_to_qcar_command rclcpp autoware_control_msgs std_msgs qcar2_interfaces)

add_executable(vehicle_interface src/vehicle_interface.cpp)
ament_target_dependencies(vehicle_interface nav_msgs autoware_control_msgs qcar2_interfaces std_msgs rclcpp)

add_executable(scan_matcher_interface src/scan_matcher_interface.cpp)
ament_target_dependencies(scan_matcher_interface nav_msgs geometry_msgs qcar2_interfaces sensor_msgs std_msgs tf2 tf2_geometry_msgs rclcpp)

install(TARGETS
    autoware_command_convert
    autoware_control_to_qcar_command
    vehicle_interface
    scan_matcher_interface
  DESTINATION lib/${PROJECT_NAME})

ament_package()
