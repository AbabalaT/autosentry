cmake_minimum_required(VERSION 3.0.2)
project(pcl2process)
#set(OpenCV_DIR “/home/ckyf/opencv/build”)

find_package(
        catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        sensor_msgs
        cv_bridge
        pcl_ros
        eigen_conversions
)
#find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES project2plane
        #  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2 tf2_ros
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/pcl2process.cpp)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
)
