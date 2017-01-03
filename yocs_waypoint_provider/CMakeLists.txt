cmake_minimum_required(VERSION 2.8.3)
project(yocs_waypoint_provider)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS visualization_msgs yocs_msgs roscpp )

# pkg-config support
find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)


catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS roscpp visualization_msgs yocs_msgs
    DEPENDS yaml-cpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})

# building c++ executables and libraries
add_library(waypoint_provider_lib src/waypoint_provider.cpp)
add_dependencies(waypoint_provider_lib yocs_msgs_gencpp)
target_link_libraries(waypoint_provider_lib ${catkin_LIBRARIES})

add_library(waypoint_provider_yaml_parser_lib src/yaml_parser.cpp)
add_dependencies(waypoint_provider_yaml_parser_lib yocs_msgs_gencpp)
target_link_libraries(waypoint_provider_yaml_parser_lib yaml-cpp ${catkin_LIBRARIES})

add_executable(waypoint_provider src/main.cpp)
add_dependencies(waypoint_provider waypoint_provider_lib waypoint_provider_yaml_parser_lib)
target_link_libraries(waypoint_provider waypoint_provider_lib waypoint_provider_yaml_parser_lib)

#############
## Install ##
#############

install(TARGETS waypoint_provider_lib waypoint_provider_yaml_parser_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS waypoint_provider 
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
