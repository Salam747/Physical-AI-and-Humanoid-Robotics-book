cmake_minimum_required(VERSION 3.8)
project(isaac_ros_perception_example)

if(CMAKE_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(isaac_ros_apriltag REQUIRED) # Example GEM dependency

ament_package()
