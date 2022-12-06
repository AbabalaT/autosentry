cmake_minimum_required(VERSION 3.0.2)
project(oak_imu)

# add_compile_options(-std=c++11)
set(ROS_BUILD_TYPE Debug)
add_definitions(-DCALIBRATION_PATH="${PROJECT_SOURCE_DIR}/params/camera/")

find_package(depthai REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(
        catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        sensor_msgs
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES spectacular_ai
        CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs depthai_bridge
        DEPENDS OpenCV
)

include_directories(
        ${depthai_bridge_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(
        ${PROJECT_NAME}_node
        src/main.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_node
        depthai::core
        depthai::opencv
        ${depthai_bridge_LIBRARIES}
        ${catkin_LIBRARIES}
)
