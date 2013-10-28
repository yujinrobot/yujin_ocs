cmake_minimum_required(VERSION 2.8.3)
project(yocs_waypoints_navi)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS move_base_msgs actionlib_msgs geometry_msgs nav_msgs actionlib roscpp tf yocs_math_toolkit)

# pkg-config support
find_package(PkgConfig)
pkg_search_module(REQUIRED yaml-cpp)

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS move_base_msgs actionlib_msgs geometry_msgs nav_msgs actionlib roscpp tf yocs_math_toolkit
    DEPENDS yaml-cpp
)

include_directories(${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})

# building c++ executables and libraries
add_executable(${PROJECT_NAME}_node src/waypoints_navi.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
