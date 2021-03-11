cmake_minimum_required(VERSION 3.0.2)
project(weeding_bot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)



## Generate services in the 'srv' folder
add_service_files(
  FILES
  nozzle_move_to_spray.srv
)



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)


catkin_install_python(PROGRAMS scripts/nozzle_move_to_spray_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
