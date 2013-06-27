cmake_minimum_required(VERSION 2.8.3)
project(yocs_diff_drive_pose_controller)

find_package(catkin REQUIRED COMPONENTS ecl_threads
                                        geometry_msgs
                                        nodelet
                                        pluginlib
                                        roscpp
                                        sensor_msgs
                                        std_msgs
                                        tf
                                        yocs_controllers)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(INCLUDE_DIRS include
               LIBRARIES yocs_diff_drive_pose_controller_nodelet
               CATKIN_DEPENDS ecl_threads
                              geometry_msgs
                              nodelet
                              pluginlib
                              roscpp
                              sensor_msgs
                              std_msgs
                              tf
                              yocs_controllers)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_library(yocs_diff_drive_pose_controller_nodelet src/nodelet.cpp)
add_dependencies(yocs_diff_drive_pose_controller_nodelet geometry_msgs_gencpp)
add_dependencies(yocs_diff_drive_pose_controller_nodelet sensor_msgs_gencpp)
add_dependencies(yocs_diff_drive_pose_controller_nodelet std_msgs_gencpp)
#target_link_libraries(yocs_diff_drive_pose_controller_nodelet ${catkin_LIBRARIES})

install(TARGETS yocs_diff_drive_pose_controller_nodelet LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)