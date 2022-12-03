cmake_minimum_required(VERSION 3.0.2)
project(spectacular_ai)

include(download.cmake)
if (EXISTS ${PROJECT_SOURCE_DIR}/thirdparty/spectacularAI/VERSION.txt)
    message(STATUS "Already have spectacularAI, don't install again. If you want to reinstall, please remove ${PROJECT_SOURCE_DIR}/thirdparty/spectacularAI Directory.")
else()
    download_and_extract(
            URL https://github.com/SpectacularAI/sdk/releases/download/v1.10.0/spectacularAI_depthaiPlugin_cpp_non-commercial_1.10.0.tar.gz
            FILENAME ${PROJECT_SOURCE_DIR}/thirdparty/spectacularAI_depthaiPlugin_cpp_non-commercial_1.10.0.tar.gz
            HASH_TYPE SHA1
            HASH cd4e4f275ef2db2a0af18d6813d398a7aef0db3f
            EXTRACT_DIR ${PROJECT_SOURCE_DIR}/thirdparty/spectacularAI
    )
endif()

# add_compile_options(-std=c++11)
set(ROS_BUILD_TYPE Debug)
add_definitions(-DCALIBRATION_PATH="${PROJECT_SOURCE_DIR}/params/camera/")

find_package(depthai REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(
        catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        sensor_msgs
        std_msgs
        image_transport
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES spectacular_ai
        CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs depthai_bridge tf2
        DEPENDS OpenCV
)

include_directories(
        include
        thirdparty/include
        ${OpenCV_INCLUDE_DIRS}
        ${tf2_ros_INCLUDE_DIRS}
        ${depthai_bridge_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(
        ${PROJECT_NAME}_node
        src/main.cpp
        src/oakInterface.cpp
        src/ros_bridge.cpp
)

target_link_directories(${PROJECT_NAME}_node PRIVATE thirdparty/lib)

target_link_libraries(
        ${PROJECT_NAME}_node
        depthai::core
        depthai::opencv
        spectacularAI_depthaiPlugin
        ${OpenCV_LIBRARIES}
        ${depthai_bridge_LIBRARIES}
        ${tf2_ros_LIBRARIES}
        ${catkin_LIBRARIES}
)
