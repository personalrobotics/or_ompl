cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED)
catkin_package(
    INCLUDE_DIRS include/
    LIBRARIES ${PROJECT_NAME}
    DEPENDS ompl openrave
)

find_package(OpenRAVE REQUIRED)
find_package(OMPL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML QUIET tinyxml)

include_directories(
    include/${PROJECT_NAME}
    ${OpenRAVE_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${TINYXML_INCLUDE_DIRS}
)
link_directories(
    ${OpenRAVE_LIBRARY_DIRS}
    ${OMPL_LIBRARY_DIRS}
)

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
            --version="${OMPL_VERSION}"
            < "${PROJECT_SOURCE_DIR}/planners.yaml"
            > "${CMAKE_BINARY_DIR}/src/PlannerRegistry.cpp"
)

# Helper library.
add_library(${PROJECT_NAME}
    src/OMPLPlanner.cpp
    src/OMPLConversions.cpp
    "${CMAKE_BINARY_DIR}/src/PlannerRegistry.cpp"
)
target_link_libraries(${PROJECT_NAME}
    ${OpenRAVE_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${TINYXML_LIBRARIES}
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS} ${OMPL_CXX_FLAGS}"
    LINK_FLAGS "${OpenRAVE_LINK_FLAGS} ${OMPL_LINK_FLAGS}"
)

# OpenRAVE plugin.
add_library(${PROJECT_NAME}_plugin SHARED
    src/OMPLMain.cpp)
target_link_libraries(${PROJECT_NAME}_plugin
    ${PROJECT_NAME} ${OpenRAVE_LIBRARIES})
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES
    PREFIX ""
    COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}"
    LINK_FLAGS "${OpenRAVE_LINK_FLAGS}"
    LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/openrave-${OpenRAVE_LIBRARY_SUFFIX}")

install(TARGETS or_ompl
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
install(TARGETS or_ompl_plugin
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}/openrave-${OpenRAVE_LIBRARY_SUFFIX}")
install(DIRECTORY "include/${PROJECT_NAME}/"
    DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
    PATTERN ".svn" EXCLUDE)
