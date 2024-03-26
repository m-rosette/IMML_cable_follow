cmake_minimum_required(VERSION 3.8)
project(dynamixel_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(dynamixel_control_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

# Build
add_executable(motor_interface src/motor_interface.cpp)
ament_target_dependencies(motor_interface
  dynamixel_sdk_custom_interfaces
  dynamixel_control_msgs
  dynamixel_sdk
  rclcpp
  std_msgs
)

add_executable(motor_interface_client src/motor_interface_client.cpp)
ament_target_dependencies(motor_interface_client
  dynamixel_sdk_custom_interfaces
  dynamixel_control_msgs
  dynamixel_sdk
  rclcpp
  std_msgs
)

add_executable(scratch_service_client src/scratch_service_client.cpp)
ament_target_dependencies(scratch_service_client
  dynamixel_sdk_custom_interfaces
  dynamixel_control_msgs
  dynamixel_sdk
  rclcpp
)

add_executable(dual_motor_client src/dual_motor_client.cpp)
ament_target_dependencies(dual_motor_client
  dynamixel_sdk_custom_interfaces
  dynamixel_control_msgs
  dynamixel_sdk
  rclcpp
)

add_executable(dual_motor_interface src/dual_motor_interface.cpp)
ament_target_dependencies(dual_motor_interface
  dynamixel_sdk_custom_interfaces
  dynamixel_control_msgs
  dynamixel_sdk
  rclcpp
)

# Install
install(TARGETS
  motor_interface
  motor_interface_client
  scratch_service_client
  dual_motor_client
  dual_motor_interface
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
