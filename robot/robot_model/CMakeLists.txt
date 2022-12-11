cmake_minimum_required(VERSION 3.0.2)
project(robot_model)

find_package(catkin REQUIRED COMPONENTS
  robot_state_publisher
  xacro
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_model
#  CATKIN_DEPENDS robot_state_publisher xacro
#  DEPENDS system_lib
)
