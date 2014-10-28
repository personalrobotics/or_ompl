cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED cmake_modules openrave_catkin)
catkin_package(
    INCLUDE_DIRS include/
    LIBRARIES ${PROJECT_NAME}
    DEPENDS ompl
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OMPL REQUIRED)
find_package(TinyXML REQUIRED)
find_package(OpenRAVE REQUIRED)

include_directories(
    include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
    ${OpenRAVE_INCLUDE_DIRS}
)
link_directories(
    ${OMPL_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
)

# Generate the OMPL planner wrappers.
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/src")

add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/src/PlannerRegistry.cpp"
    MAIN_DEPENDENCY "${PROJECT_SOURCE_DIR}/planners.yaml"
    DEPENDS "${PROJECT_SOURCE_DIR}/scripts/wrap_planners.py"
    COMMAND "${PROJECT_SOURCE_DIR}/scripts/wrap_planners.py"
            --include-dirs="${OMPL_INCLUDE_DIRS}"
            < "${PROJECT_SOURCE_DIR}/planners.yaml"
            > "${CMAKE_BINARY_DIR}/src/PlannerRegistry.cpp"
)

# Helper library.
add_library(${PROJECT_NAME}
    src/OMPLPlanner.cpp
    src/OMPLSimplifier.cpp
    src/OMPLConversions.cpp
    "${CMAKE_BINARY_DIR}/src/PlannerRegistry.cpp"
)
target_link_libraries(${PROJECT_NAME}
    ${OpenRAVE_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${TinyXML_LIBRARIES}
)

# OpenRAVE plugin.
openrave_plugin("${PROJECT_NAME}_plugin"
    src/OMPLMain.cpp
)
target_link_libraries("${PROJECT_NAME}_plugin"
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

install(TARGETS or_ompl
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
)
install(DIRECTORY "include/${PROJECT_NAME}/"
    DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
    PATTERN ".svn" EXCLUDE
)
