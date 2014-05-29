##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(yocs_ar_marker_tracking)

##############################################################################
# Find
##############################################################################

find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  roscpp
  geometry_msgs
  sensor_msgs
  yocs_math_toolkit
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig)
pkg_search_module(REQUIRED yaml_cpp)

##############################################################################
# Catkin
##############################################################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yocs_ar_marker_tracking
  CATKIN_DEPENDS ar_track_alvar roscpp geometry_msgs sensor_msgs yocs_math_toolkit
  DEPENDS Boost yaml-cpp
)

##############################################################################
# Build
##############################################################################

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})

add_subdirectory(src)

##############################################################################
# Install
##############################################################################

# TODO
