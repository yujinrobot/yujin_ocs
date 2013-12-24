cmake_minimum_required(VERSION 2.8.3)
project(yocs_waypoints_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  yocs_msgs
)

catkin_package()
catkin_python_setup()

install(PROGRAMS 
  scripts/yocs_waypoints_manager 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

