##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.4.6)

##############################################################################
# Ros
##############################################################################

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

##############################################################################
# Dynamic Reconfigure
##############################################################################

find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(cfg/Reload.cfg)

##############################################################################
# Nodelet
##############################################################################

rosbuild_add_library(cmd_vel_mux_nodelet src/cmd_vel_mux_nodelet.cpp src/cmd_vel_subscribers.cpp)
target_link_libraries(cmd_vel_mux_nodelet yaml-cpp)
