cmake_minimum_required(VERSION 2.8.3)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS openrave ompl
)

include(FindPkgConfig)
pkg_check_modules(OPENRAVE REQUIRED openrave0.9)
pkg_check_modules(OMPL REQUIRED ompl)

include_directories(
    include/${PROJECT_NAME}
    ${OPENRAVE_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS})
link_directories(${OPENRAVE_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/OMPLPlanner.cpp)
target_link_libraries(${PROJECT_NAME}
    ${OPENRAVE_LIBRARIES} ${OMPL_LIBRARIES})

add_library(${PROJECT_NAME}_plugin src/OMPLMain.cpp)
target_link_libraries(${PROJECT_NAME}_plugin
    ${OPENRAVE_LIBRARIES} ${OMPL_LIBRARIES})
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES PREFIX "")

install(TARGETS or_ompl
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS or_ompl_plugin
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/openrave0.9)
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE)
