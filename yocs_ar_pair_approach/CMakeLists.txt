##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(yocs_ar_pair_approach)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

##############################################################################
# Installs
##############################################################################

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  PROGRAMS
    scripts/approach.py
    scripts/disable_service
    scripts/enable_service
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
