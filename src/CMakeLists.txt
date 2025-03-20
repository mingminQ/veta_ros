# ================================================================================
# AUTHOR : Minkyu Kil
# FILE   : veta_ros/src/veta/CMakeLists.txt
# CAPTION: VETA - Vehicle Environment Trajectory Algorithms
# ================================================================================
cmake_minimum_required(VERSION 3.0.2)
project(veta_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Catkin build package
find_package(Eigen3 REQUIRED)
find_package( 
    catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        tf2
        tf2_ros
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

catkin_package(

    INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIRS}

    LIBRARIES veta_ros
        veta_util
        veta_math
        veta_space
        veta_noetic

    CATKIN_DEPENDS
        roscpp

    DEPENDS 
        EIGEN3
)

# VETA core algorithms and utilities
add_subdirectory(veta)
add_subdirectory(veta_noetic)

add_library(veta_ros INTERFACE)
target_link_libraries( veta_ros INTERFACE
    veta_util
    veta_math
    veta_space
    veta_noetic
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)