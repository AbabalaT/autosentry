cmake_minimum_required(VERSION 3.0.2)
project(robot_driver)

find_package(
        catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        message_generation
        message_runtime
        roscpp
        serial
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
# )

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES robot_driver
        #  CATKIN_DEPENDS geometry_msgs message_generation message_runtime roscpp serial
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/serialPort.cpp)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
)
