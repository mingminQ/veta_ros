# ================================================================================
# AUTHOR : Minkyu Kil
# FILE   : veta_ros/src/veta/CMakeLists.txt
# CAPTION: veta_ros sub-library CMake file
# ================================================================================

# veta_util ----------------------------------------------------------------------
file(GLOB_RECURSE VETA_UTIL_GENERAL util/src/*.cpp)
file(GLOB_RECURSE VETA_UTIL_GENERIC util/generic/src/*.cpp)

add_library( veta_util
    ${VETA_UTIL_GENERAL}
    ${VETA_UTIL_GENERIC}
)
target_include_directories( veta_util 
    PUBLIC
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries( veta_util 
    PUBLIC
        ${catkin_LIBRARIES}
)

# veta_math ----------------------------------------------------------------------
file(GLOB_RECURSE VETA_MATH_GENERAL    math/src/*.cpp           )
file(GLOB_RECURSE VETA_MATH_RANDOM     math/random/src/*.cpp    )
file(GLOB_RECURSE VETA_MATH_POLYNOMIAL math/polynomial/src/*.cpp)

add_library( veta_math 
    ${VETA_MATH_GENERAL}
    ${VETA_MATH_RANDOM}
    ${VETA_MATH_POLYNOMIAL}
)

target_include_directories( veta_math
    PUBLIC 
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)

target_link_libraries( veta_math
    PUBLIC
        ${catkin_LIBRARIES}
    PRIVATE
        ${EIGEN3_LIBRARIES}
        veta_util
)

# veta_space ---------------------------------------------------------------------
file(GLOB_RECURSE VETA_SPACE_GENERAL space/src/*.cpp      )
file(GLOB_RECURSE VETA_SPACE_BASE    space/base/src/*.cpp )

add_library( veta_space
    ${VETA_SPACE_GENERAL}
    ${VETA_SPACE_BASE}
)

target_include_directories( veta_space
    PUBLIC 
        ${catkin_INCLUDE_DIRS}
)

target_link_libraries( veta_space
    PUBLIC
        ${catkin_LIBRARIES}
    PRIVATE
        veta_util
)