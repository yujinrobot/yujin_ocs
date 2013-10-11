cmake_minimum_required(VERSION 2.8.3)
project(yocs_cmd_vel_mux)
find_package(catkin REQUIRED COMPONENTS roscpp pluginlib nodelet dynamic_reconfigure geometry_msgs)

# pkg-config support
find_package(PkgConfig)
pkg_search_module(REQUIRED yaml-cpp)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/reload.cfg)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}_nodelet
   CATKIN_DEPENDS roscpp pluginlib nodelet dynamic_reconfigure geometry_msgs
   DEPENDS yaml-cpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})

# Nodelet library
add_library(${PROJECT_NAME}_nodelet src/cmd_vel_mux_nodelet.cpp src/cmd_vel_subscribers.cpp)
add_dependencies(${PROJECT_NAME}_nodelet geometry_msgs_gencpp)
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_nodelet yaml-cpp)

install(TARGETS ${PROJECT_NAME}_nodelet
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
