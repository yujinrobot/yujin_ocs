##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(yocs_ar_pair_tracking)

##############################################################################
# Find
##############################################################################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  yocs_math_toolkit
  yocs_ar_marker_tracking
  ar_track_alvar_msgs
  yocs_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

##############################################################################
# Catkin
##############################################################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yocs_ar_pair_tracking
  CATKIN_DEPENDS roscpp ar_track_alvar_msgs std_msgs geometry_msgs yocs_math_toolkit yocs_ar_marker_tracking yocs_msgs
  DEPENDS Boost
)

##############################################################################
# Build
##############################################################################

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_subdirectory(src)

##############################################################################
# Install
##############################################################################

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
