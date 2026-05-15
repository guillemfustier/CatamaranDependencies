cmake_minimum_required(VERSION 3.8)
project(catamaran_camera_republisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(adaptive_camera_republisher src/adaptive_camera_republisher.cpp)
add_executable(adaptive_lidar_republisher src/adaptive_lidar_republisher.cpp)

target_include_directories(adaptive_camera_republisher PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(adaptive_lidar_republisher PRIVATE
  include
)

ament_target_dependencies(adaptive_camera_republisher
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(adaptive_lidar_republisher
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(adaptive_camera_republisher
  ${OpenCV_LIBS}
)

if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(adaptive_camera_republisher yaml-cpp::yaml-cpp)
  target_link_libraries(adaptive_lidar_republisher yaml-cpp::yaml-cpp)
else()
  target_link_libraries(adaptive_camera_republisher yaml-cpp)
  target_link_libraries(adaptive_lidar_republisher yaml-cpp)
endif()

install(TARGETS
  adaptive_camera_republisher
  adaptive_lidar_republisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
