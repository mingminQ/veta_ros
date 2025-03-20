# ================================================================================
# AUTHOR : Minkyu Kil
# FILE   : veta_ros/demo/util/CMakeLists.txt
# CAPTION: veta_ros utility demo node CMake file
# ================================================================================

# Define file path ---------------------------------------------------------------

file(GLOB VETA_ROS_DEMO src)

# Executable node files ----------------------------------------------------------

# [Transform Util]
add_executable(TransformDemo ${VETA_ROS_DEMO}/TransformDemo.cpp)
target_link_libraries(TransformDemo ${catkin_LIBRARIES} veta_ros)

# --------------------------------------------------------------------------------