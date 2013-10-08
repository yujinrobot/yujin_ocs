cmake_minimum_required(VERSION 2.8.3)
project(yocs_virtual_sensor)

## Find catkin macros and libraries
find_package(catkin REQUIRED tf roscpp sensor_msgs geometry_msgs yocs_msgs yocs_math_toolkit)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES virtual_sensor
  CATKIN_DEPENDS tf roscpp sensor_msgs geometry_msgs yocs_msgs yocs_math_toolkit
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(virtual_sensor_node src/virtual_sensor_node.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(virtual_sensor_node yocs_msgs_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(virtual_sensor_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS virtual_sensor_node
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# TODO
