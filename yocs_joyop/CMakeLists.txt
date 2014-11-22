cmake_minimum_required(VERSION 2.8.3)
project(yocs_joyop)

find_package(catkin REQUIRED COMPONENTS ecl_exceptions
                                        ecl_time
                                        geometry_msgs
                                        sensor_msgs
                                        std_msgs
                                        roscpp)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(joyop src/joyop.cpp)

target_link_libraries(joyop ${catkin_LIBRARIES})

install(TARGETS joyop 
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
