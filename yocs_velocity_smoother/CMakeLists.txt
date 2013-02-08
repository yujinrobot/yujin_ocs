cmake_minimum_required(VERSION 2.8.3)
project(yocs_velocity_smoother)
find_package(catkin REQUIRED COMPONENTS roscpp pluginlib nodelet ecl_threads dynamic_reconfigure geometry_msgs nav_msgs)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/params.cfg)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES velocity_smoother_nodelet
   CATKIN_DEPENDS roscpp pluginlib nodelet ecl_threads dynamic_reconfigure geometry_msgs nav_msgs
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)


# Nodelet library
add_library(velocity_smoother_nodelet src/velocity_smoother_nodelet.cpp)


target_link_libraries(velocity_smoother_nodelet ${catkin_LIBRARIES})

install(TARGETS velocity_smoother_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/velocity_smoother/
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
