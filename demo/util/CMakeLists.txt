# ================================================================================
# AUTHOR : Minkyu Kil
# FILE   : veta_ros/demo/util/CMakeLists.txt
# CAPTION: veta_ros utility demo node CMake file
# ================================================================================

# Define file path ---------------------------------------------------------------

file(GLOB VETA_UTIL_DEMO src)
#file(GLOB VETA_UTIL_GENERIC_DEMO src)

# Executable node files ----------------------------------------------------------

# [Output handler]
add_executable(LogHandlerDemo ${VETA_UTIL_DEMO}/LogHandlerDemo.cpp)
target_link_libraries(LogHandlerDemo ${catkin_LIBRARIES} veta_ros)

# [Exception]
add_executable(ExceptionDemo ${VETA_UTIL_DEMO}/ExceptionDemo.cpp)
target_link_libraries(ExceptionDemo ${catkin_LIBRARIES} veta_ros)

# [Generic parameter]
add_executable(GenericParameterDemo ${VETA_UTIL_DEMO}/GenericParameterDemo.cpp)
target_link_libraries(GenericParameterDemo ${catkin_LIBRARIES} veta_ros)

# --------------------------------------------------------------------------------