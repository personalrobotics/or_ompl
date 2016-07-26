cmake_minimum_required(VERSION 2.8.3)
project(or_ompl)

include(CheckCXXSourceCompiles)

find_package(catkin REQUIRED cmake_modules openrave_catkin)
find_package(Boost REQUIRED COMPONENTS chrono system)
find_package(OMPL REQUIRED)
find_package(OpenRAVE REQUIRED)
find_package(TinyXML REQUIRED)
find_package(Eigen REQUIRED)

# 2013-01-05: int options arg added to PlannerParameters::serialize
set(CMAKE_REQUIRED_INCLUDES ${OpenRAVE_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES ${OpenRAVE_LIBRARIES} ${Boost_LIBRARIES})
check_cxx_source_compiles(
    "#include <openrave/openrave.h>
    class P: OpenRAVE::PlannerBase::PlannerParameters
    {void f(){bool (P::*x)(std::ostream&,int) const = &P::serialize;}};
    int main(){}"
    OR_OMPL_HAS_PPSEROPTS)

# 1.2.0 ompl (ROS kinetic) switched from boost to std smart pointers
set(CMAKE_REQUIRED_INCLUDES ${OMPL_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES "")
check_cxx_source_compiles(
    "#include <ompl/base/StateSpace.h>
    int main(){ ompl::base::StateSpacePtr s = boost::shared_ptr<ompl::base::StateSpace>(); }"
    OR_OMPL_HAS_BOOSTSMARTPTRS)

configure_file(
    include/${PROJECT_NAME}/config.h.in
    ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/config.h
)

catkin_package(
    INCLUDE_DIRS include ${CATKIN_DEVEL_PREFIX}/include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS Boost Eigen OMPL OpenRAVE
)

if(NOT OR_OMPL_HAS_BOOSTSMARTPTRS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(NOT OR_OMPL_HAS_BOOSTSMARTPTRS)

include_directories(
    include
    ${CATKIN_DEVEL_PREFIX}/include
    ${Boost_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${OpenRAVE_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
link_directories(
    ${OMPL_LIBRARY_DIRS}
    ${OpenRAVE_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
)
add_definitions(
    ${Eigen_DEFINITIONS}
)

# Generate the OMPL planner wrappers.
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/src")

add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/src/PlannerRegistry.cpp"
    MAIN_DEPENDENCY "${PROJECT_SOURCE_DIR}/planners.yaml"
    DEPENDS "${PROJECT_SOURCE_DIR}/scripts/wrap_planners.py"
    COMMAND "${PROJECT_SOURCE_DIR}/scripts/wrap_planners.py"
            --include-dirs="${OMPL_INCLUDE_DIRS}"
            --planners-yaml="${PROJECT_SOURCE_DIR}/planners.yaml"
            --generated-cpp="${CMAKE_CURRENT_BINARY_DIR}/src/PlannerRegistry.cpp"
)

# Helper library.
add_library(${PROJECT_NAME}
    src/OMPLConversions.cpp
    src/OMPLPlanner.cpp
    src/OMPLSimplifier.cpp
    src/StateSpaces.cpp
    src/TSR.cpp
    src/TSRChain.cpp
    src/TSRGoal.cpp
    src/TSRRobot.cpp
    "${CMAKE_CURRENT_BINARY_DIR}/src/PlannerRegistry.cpp"
)
target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${Eigen_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${OpenRAVE_LIBRARIES}
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
    PATTERN "*.in" EXCLUDE
    PATTERN ".svn" EXCLUDE
)
install(DIRECTORY
    "${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/"
    DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)

# Tests
if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(tests/test_Planner.py)
endif(CATKIN_ENABLE_TESTING)
