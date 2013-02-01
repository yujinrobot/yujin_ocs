cmake_minimum_required(VERSION 2.8.3)
project(yocs_controllers)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp 
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
