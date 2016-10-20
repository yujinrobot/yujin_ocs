##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.4)
project(yocs_joyop)

##############################################################################
# Dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS ecl_exceptions
                                        ecl_time
                                        geometry_msgs
                                        sensor_msgs
                                        std_msgs
                                        yocs_msgs
                                        roscpp
                                        yocs_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

##############################################################################
# Catkin
##############################################################################

catkin_package()

##############################################################################
# Build
##############################################################################

include_directories(${catkin_INCLUDE_DIRS})
add_executable(joyop src/joyop.cpp)
target_link_libraries(joyop ${catkin_LIBRARIES})

##############################################################################
# Install
##############################################################################

install(PROGRAMS
            scripts/magic_button_relay.py
            scripts/reconfigure_joystick.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(TARGETS
            joyop 
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
