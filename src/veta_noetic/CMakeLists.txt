# ================================================================================
# AUTHOR : Minkyu Kil
# FILE   : veta_ros/src/veta_noetic/CMakeLists.txt
# CAPTION: veta_ros noetic distro utiliies
# ================================================================================

# veta_noetic --------------------------------------------------------------------
file(GLOB_RECURSE VETA_NOETIC_TRANSFORM transform/src/*.cpp)
file(GLOB_RECURSE VETA_NOETIC_RVIZ      rviz/src/*.cpp     )

add_library( veta_noetic
    ${VETA_NOETIC_TRANSFORM}
    ${VETA_NOETIC_RVIZ}
)

target_include_directories( veta_noetic
    PUBLIC 
        ${catkin_INCLUDE_DIRS}

)

target_link_libraries( veta_noetic
    PUBLIC
        ${catkin_LIBRARIES}
    PRIVATE
        veta_util
)