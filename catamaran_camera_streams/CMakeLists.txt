cmake_minimum_required(VERSION 3.8)
project(catamaran_camera_streams)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(camera_compressed_publisher src/camera_compressed_publisher.cpp)

target_include_directories(camera_compressed_publisher PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camera_compressed_publisher
  ${OpenCV_LIBS}
)

ament_target_dependencies(camera_compressed_publisher
  rclcpp
  sensor_msgs
)

install(TARGETS
  camera_compressed_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  udev
  DESTINATION share/${PROJECT_NAME}
)

ament_package()