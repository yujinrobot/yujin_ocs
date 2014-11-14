cmake_minimum_required(VERSION 2.8.3)
project(yocs_rapps)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY rapps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
