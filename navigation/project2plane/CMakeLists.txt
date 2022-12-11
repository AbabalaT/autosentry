cmake_minimum_required(VERSION 3.0.2)
project(project2plane)

find_package(
        catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        sensor_msgs
        tf2
        tf2_ros
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES project2plane
        #  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2 tf2_ros
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/laserProjection.cpp)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
)
