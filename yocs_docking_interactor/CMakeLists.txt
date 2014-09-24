cmake_minimum_required(VERSION 2.8.3)
project(yocs_docking_interactor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  yocs_math_toolkit
  yocs_ar_marker_tracking
  dynamic_reconfigure
  yocs_navigator
  tf
  ar_track_alvar_msgs
  yocs_msgs
  move_base_msgs
  kobuki_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yocs_docking_interactor yocs_docking_ar_tracker
  CATKIN_DEPENDS roscpp yocs_math_toolkit actionlib move_base_msgs yocs_msgs ar_track_alvar_msgs  yocs_ar_marker_tracking kobuki_msgs
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(yocs_docking_ar_tracker
  src/${PROJECT_NAME}/docking_ar_tracker.cpp
  src/${PROJECT_NAME}/tracker_handles.cpp
)
target_link_libraries(yocs_docking_ar_tracker ${catkin_LIBRARIES})
add_dependencies(yocs_docking_ar_tracker yocs_msgs_gencpp)

add_library(yocs_docking_interactor
  src/${PROJECT_NAME}/docking_interactor.cpp
  src/${PROJECT_NAME}/utils.cpp
  src/${PROJECT_NAME}/command_handles.cpp
  src/${PROJECT_NAME}/return_to_dock_handles.cpp
)
target_link_libraries(yocs_docking_interactor yocs_docking_ar_tracker ${catkin_LIBRARIES})
add_dependencies(yocs_docking_interactor yocs_msgs_gencpp kobuki_msgs_gencpp)

## Declare a cpp executable
add_executable(docking_interactor_node src/docking_interactor_main.cpp)
target_link_libraries(docking_interactor_node yocs_docking_interactor)

install(TARGETS yocs_docking_interactor docking_interactor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
