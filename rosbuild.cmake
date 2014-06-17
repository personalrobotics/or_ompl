cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

include_directories(${PROJECT_SOURCE_DIR}/include/or_ompl)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Generate the OMPL planner wrappers.
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/src")
add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/src/PlannerRegistry.cpp"
    MAIN_DEPENDENCY "${PROJECT_SOURCE_DIR}/planners.json"
    DEPENDS "${PROJECT_SOURCE_DIR}/scripts/wrap_planners.py"
    COMMAND "${PROJECT_SOURCE_DIR}/scripts/wrap_planners.py"
            < "${PROJECT_SOURCE_DIR}/planners.json"
            > "${CMAKE_BINARY_DIR}/src/PlannerRegistry.cpp"
)

rosbuild_add_library(${PROJECT_NAME} SHARED
    src/OMPLMain.cpp
    src/OMPLPlanner.cpp
    "${CMAKE_BINARY_DIR}/src/PlannerRegistry.cpp"
)
target_link_libraries(${PROJECT_NAME} ompl)
