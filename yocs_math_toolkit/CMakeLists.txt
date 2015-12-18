cmake_minimum_required(VERSION 2.8.3)
project(yocs_math_toolkit)

##############################################################################
# Find Packages
##############################################################################

find_package(catkin
    REQUIRED
    COMPONENTS
        ecl_build
        ecl_config
        ecl_exceptions
        ecl_formatters
        ecl_linear_algebra
        geometry_msgs
        roscpp
        tf
)

##############################################################################
# Exports
##############################################################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        ecl_build
        ecl_config
        ecl_exceptions
        ecl_formatters
        ecl_linear_algebra
        geometry_msgs
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
# Sources
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

