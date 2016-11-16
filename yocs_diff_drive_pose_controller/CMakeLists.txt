cmake_minimum_required(VERSION 2.8.3)
project(yocs_diff_drive_pose_controller)

find_package(catkin REQUIRED COMPONENTS ecl_threads
                                        dynamic_reconfigure
                                        geometry_msgs
                                        nodelet
                                        pluginlib
                                        roscpp
                                        sensor_msgs
                                        std_msgs
                                        tf
                                        yocs_controllers
                                        yocs_math_toolkit
                                        yocs_msgs
            )

catkin_package(INCLUDE_DIRS include
               LIBRARIES yocs_diff_drive_pose_controller yocs_diff_drive_pose_controller_nodelet
               CATKIN_DEPENDS ecl_threads
                              dynamic_reconfigure
                              geometry_msgs
                              nodelet
                              pluginlib
                              roscpp
                              sensor_msgs
                              std_msgs
                              tf
                              yocs_controllers
                              yocs_math_toolkit
                              yocs_msgs
              )

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(yocs_diff_drive_pose_controller src/diff_drive_pose_controller.cpp)
target_link_libraries(yocs_diff_drive_pose_controller ${catkin_LIBRARIES})

add_library(yocs_diff_drive_pose_controller_nodelet
    src/diff_drive_pose_controller.cpp
    src/diff_drive_pose_controller_ros.cpp
    src/nodelet.cpp)
add_dependencies(yocs_diff_drive_pose_controller_nodelet geometry_msgs_gencpp
                                                         sensor_msgs_gencpp
                                                         std_msgs_gencpp)
target_link_libraries(yocs_diff_drive_pose_controller_nodelet yocs_diff_drive_pose_controller ${catkin_LIBRARIES})

install(TARGETS yocs_diff_drive_pose_controller yocs_diff_drive_pose_controller_nodelet LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY plugins DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

