cmake_minimum_required(VERSION 2.8.3)
project(yocs_waypoints_navi)

find_package(catkin REQUIRED COMPONENTS actionlib
                                        actionlib_msgs
                                        geometry_msgs
                                        move_base_msgs
                                        nav_msgs
                                        roscpp
                                        tf
                                        visualization_msgs
                                        yocs_math_toolkit
                                        yocs_msgs)

catkin_package()

include_directories(include
                    ${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_node src/waypoints_navi.cpp
                                    src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
                                           ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node yocs_msgs_generate_messages_cpp)

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
