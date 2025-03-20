# VETA : Vehicle Environment Trajectory Algorithms
cmake_minimum_required(VERSION 3.0.2)
project(veta_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package( catkin REQUIRED COMPONENTS 
    roscpp
    veta_ros
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_package( CATKIN_DEPENDS
    roscpp
    veta_ros
)

# VETA library demo sub-directory
add_subdirectory(util)
add_subdirectory(math)
add_subdirectory(space)
add_subdirectory(noetic)
