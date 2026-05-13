cmake_minimum_required(VERSION 3.8)
project(motor_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

include_directories(include)

add_executable(rudder_angle_controller src/rudder_angle_controller.cpp)
add_executable(rudder_angle_controller_abs src/rudder_angle_controller_abs.cpp)
add_executable(cable_distance_controller src/cable_distance_controller.cpp)
add_executable(lars_height_controller src/lars_height_controller.cpp)
add_executable(rudder_angle_publisher src/rudder_angle_publisher.cpp)
add_executable(cmdvel_motor_controller src/cmdvel_motor_controller.cpp)
add_executable(cmdvel_mavlink_controller src/cmdvel_mavlink_controller.cpp)

ament_target_dependencies(rudder_angle_controller 
  rclcpp
  dynamixel_sdk
  custom_interfaces
  geometry_msgs
  std_msgs
)

ament_target_dependencies(rudder_angle_controller_abs 
  rclcpp
  dynamixel_sdk
  custom_interfaces
  geometry_msgs
  std_msgs
)

ament_target_dependencies(cable_distance_controller
  rclcpp
  dynamixel_sdk
  std_msgs
)

ament_target_dependencies(lars_height_controller
  rclcpp
  dynamixel_sdk
  std_msgs
)

ament_target_dependencies(rudder_angle_publisher 
  rclcpp
  std_msgs
)

ament_target_dependencies(cmdvel_motor_controller 
  rclcpp 
  geometry_msgs
  mavros_msgs
  std_msgs
)

ament_target_dependencies(cmdvel_mavlink_controller 
  rclcpp 
  geometry_msgs
  std_msgs
)


install (TARGETS
  rudder_angle_controller
  rudder_angle_controller_abs
  cable_distance_controller
  lars_height_controller
  rudder_angle_publisher
  cmdvel_motor_controller
  cmdvel_mavlink_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
