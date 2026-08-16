cmake_minimum_required(VERSION 3.8)
project(assignment_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    assignment_navigation/cmd_vel_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
    maps
    rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
