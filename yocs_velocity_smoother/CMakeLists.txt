cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

##############################################################################
# Dynamic Reconfigure
##############################################################################

# Note, this initialises CMAKE_LIBRARY_OUTPUT_DIRECTORY, take care below.
find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(cfg/Config.cfg)

##############################################################################
# NODELET
##############################################################################

rosbuild_add_library(velocity_smoother_nodelet src/velocity_smoother_nodelet.cpp)
set_target_properties(velocity_smoother_nodelet PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
