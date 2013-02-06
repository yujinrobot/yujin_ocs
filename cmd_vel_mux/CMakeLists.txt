cmake_minimum_required(VERSION 2.8.3)
project(cmd_vel_mux)
find_package(catkin REQUIRED COMPONENTS roscpp nodelet dynamic_reconfigure)

# pkg-config support
find_package(PkgConfig)
pkg_search_module(REQUIRED yaml-cpp)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/reload.cfg)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cmd_vel_mux_nodelet
   CATKIN_DEPENDS roscpp nodelet dynamic_reconfigure 
   DEPENDS yaml-cpp
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)


# Nodelet library
add_library(cmd_vel_mux_nodelet src/cmd_vel_mux_nodelet.cpp src/cmd_vel_subscribers.cpp)

target_link_libraries(cmd_vel_mux_nodelet ${catkin_LIBRARIES})
target_link_libraries(cmd_vel_mux_nodelet yaml-cpp)

install(TARGETS cmd_vel_mux_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
