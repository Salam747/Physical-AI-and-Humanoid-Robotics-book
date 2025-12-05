cmake_minimum_required(VERSION 3.8)
project(humanoid_launch)

if(CMAKE_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
