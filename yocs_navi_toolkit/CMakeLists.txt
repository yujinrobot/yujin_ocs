##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(yocs_navi_toolkit)

##############################################################################
# Find Packages
##############################################################################

find_package(
    catkin
        REQUIRED
        COMPONENTS
            costmap_2d
            nav_msgs
            ecl_build
            roscpp
            tf
)

##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
add_definitions(${catkin_DEFINITIONS})
include_directories(include ${catkin_INCLUDE_DIRS})

##############################################################################
# Catkin
##############################################################################

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        costmap_2d
        nav_msgs
        ecl_build
        roscpp
        tf
)

##############################################################################
# Sources
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

##############################################################################
# Installs
##############################################################################

#install(DIRECTORY plugins DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
