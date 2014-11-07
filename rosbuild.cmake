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

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
    ${OMPL_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
)

find_package(PkgConfig)
pkg_check_modules(OMPL QUIET ompl)
pkg_check_modules(TinyXML QUIET tinyxml)

# Generate the OMPL planner wrappers.
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/src")

execute_process(COMMAND pkg-config ompl --modversion
    OUTPUT_VARIABLE OMPL_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/src/PlannerRegistry.cpp"
    MAIN_DEPENDENCY "${PROJECT_SOURCE_DIR}/planners.yaml"
    DEPENDS "${PROJECT_SOURCE_DIR}/scripts/wrap_planners.py"
    COMMAND "${PROJECT_SOURCE_DIR}/scripts/wrap_planners.py"
            --include-dirs="${OMPL_INCLUDE_DIRS}"
            < "${PROJECT_SOURCE_DIR}/planners.yaml"
            > "${CMAKE_BINARY_DIR}/src/PlannerRegistry.cpp"
)

rosbuild_add_library(${PROJECT_NAME} SHARED
    src/OMPLMain.cpp
    src/OMPLPlanner.cpp
    src/OMPLSimplifier.cpp
    src/OMPLConversions.cpp
    "${CMAKE_BINARY_DIR}/src/PlannerRegistry.cpp"
)
target_link_libraries(${PROJECT_NAME}
    ${OMPL_LIBRARIES}
    ${TinyXML_LIBRARIES}
)
