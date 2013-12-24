cmake_minimum_required(VERSION 2.8.3)
project(yocs_waypoints_manager)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS visualization_msgs geometry_msgs roscpp )

# pkg-config support
find_package(PkgConfig)
pkg_search_module(REQUIRED yaml-cpp)

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS visualization_msgs geometry_msgs roscpp
    DEPENDS yaml-cpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})

# building c++ executables and libraries
add_library(waypoint_manager_lib src/waypoints_manager.cpp)
add_dependencies(waypoint_manager_lib geometry_msgs_gencpp)
target_link_libraries(waypoint_manager_lib ${catkin_LIBRARIES} yaml-cpp)

add_executable(waypoint_manager src/main.cpp)
add_dependencies(waypoint_manager waypoint_manager_lib)
target_link_libraries(waypoint_manager waypoint_manager_lib)

#############
## Install ##
#############

install(TARGETS waypoint_manager 
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
