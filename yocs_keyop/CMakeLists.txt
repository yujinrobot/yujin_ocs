cmake_minimum_required(VERSION 2.8.3)
project(yocs_keyop)
find_package(catkin REQUIRED COMPONENTS ecl_exceptions
                                        ecl_threads
                                        ecl_time
                                        geometry_msgs
                                        std_msgs
                                        roscpp)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(src)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
