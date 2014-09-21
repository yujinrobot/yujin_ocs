cmake_minimum_required(VERSION 2.8.3)
project(yocs_docking_interactor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  yocs_math_toolkit
  dynamic_reconfigure
)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yocs_docking_interactor
  CATKIN_DEPENDS roscpp yocs_math_toolkit
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(yocs_docking_interactor
  src/${PROJECT_NAME}/docking_interactor.cpp
  src/${PROJECT_NAME}/logs.cpp
  src/${PROJECT_NAME}/tracker_handles.cpp
)
target_link_libraries(yocs_docking_interactor ${catkin_LIBRARIES})
add_dependencies(yocs_docking_interactor yocs_msgs_gencpp)

## Declare a cpp executable
add_executable(docking_interactor_node src/docking_interactor_main.cpp)
target_link_libraries(docking_interactor_node yocs_docking_interactor)

install(TARGETS yocs_docking_interactor docking_interactor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
