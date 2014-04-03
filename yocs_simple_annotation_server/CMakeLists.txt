cmake_minimum_required(VERSION 2.8.3)
project(yocs_simple_annotation_server)

find_package(catkin REQUIRED COMPONENTS roscpp yocs_msgs)

# pkg-config support
find_package(PkgConfig)
pkg_search_module(REQUIRED yaml-cpp)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp yocs_msgs
  DEPENDS yaml-cpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})
