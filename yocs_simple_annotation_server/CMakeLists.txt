cmake_minimum_required(VERSION 2.8.3)
project(yocs_simple_annotation_server)

find_package(catkin REQUIRED COMPONENTS rospy ar_track_alvar yocs_msgs)

catkin_package()
catkin_python_setup()

install(PROGRAMS scripts/simple_annotation_server.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
