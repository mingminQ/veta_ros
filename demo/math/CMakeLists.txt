# ================================================================================
# AUTHOR : Minkyu Kil
# FILE   : veta_ros/demo/math/CMakeLists.txt
# CAPTION: veta_ros utility demo node CMake file
# ================================================================================

# Define file path ---------------------------------------------------------------
file(GLOB VETA_MATH_DEMO src)

# Executable node files ----------------------------------------------------------

# [Randomizer]
add_executable(RandomizerDemo ${VETA_MATH_DEMO}/RandomizerDemo.cpp)
target_link_libraries(RandomizerDemo ${catkin_LIBRARIES} veta_ros)

# --------------------------------------------------------------------------------