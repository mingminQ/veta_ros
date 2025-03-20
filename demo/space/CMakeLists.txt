# ================================================================================
# AUTHOR : Minkyu Kil
# FILE   : veta_ros/demo/state/CMakeLists.txt
# CAPTION: veta_ros state data type demo node CMake file
# ================================================================================

# Define file path ---------------------------------------------------------------

file(GLOB VETA_STATE_DEMO src)

# Executable node files ----------------------------------------------------------

# [State space demo]
add_executable(StateSpaceDemo ${VETA_STATE_DEMO}/StateSpaceDemo.cpp)
target_link_libraries(StateSpaceDemo ${catkin_LIBRARIES} veta_ros)

# --------------------------------------------------------------------------------