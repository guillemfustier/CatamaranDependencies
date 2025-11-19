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

include_directories(include)

add_executable(rudder_angle_controller src/rudder_angle_controller.cpp)

ament_target_dependencies(rudder_angle_controller 
  rclcpp
  dynamixel_sdk
  custom_interfaces
  geometry_msgs
  std_msgs
)

install (TARGETS
  rudder_angle_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
