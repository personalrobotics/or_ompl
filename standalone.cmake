cmake_minimum_required(VERSION 2.8.3)
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib")

find_package(Boost REQUIRED COMPONENTS system)
find_package(OMPL REQUIRED)
find_package(OpenRAVE REQUIRED)

find_package(PkgConfig)
pkg_check_modules(TinyXML REQUIRED QUIET tinyxml)

include_directories(
    include/${PROJECT_NAME}
    ${OMPL_INCLUDE_DIRS}
    ${OpenRAVE_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
)
link_directories(
    ${OMPL_LIBRARY_DIRS}
    ${OpenRAVE_LIBRARY_DIRS}
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
add_library(${PROJECT_NAME} SHARED
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
add_library("${PROJECT_NAME}_plugin" SHARED
    src/OMPLMain.cpp
)
set_target_properties("${PROJECT_NAME}_plugin" PROPERTIES
    PREFIX ""
    COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}"
    LINK_FLAGS "${OpenRAVE_LINK_FLAGS}"
)
target_link_libraries("${PROJECT_NAME}_plugin"
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
)
