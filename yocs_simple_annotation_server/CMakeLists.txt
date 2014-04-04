cmake_minimum_required(VERSION 2.8.3)
project(yocs_simple_annotation_server)

find_package(catkin REQUIRED COMPONENTS roscpp yocs_msgs)

# pkg-config support
find_package(PkgConfig)
pkg_search_module(REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp yocs_msgs
  DEPENDS yaml-cpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})


add_library(yocs_simple_annotation_server src/simple_annotation_server.cpp 
                                          src/file_io.cpp)
add_dependencies(yocs_simple_annotation_server yocs_msgs_gencpp)
target_link_libraries(yocs_simple_annotation_server ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES})

add_executable(simple_annotation_server src/main.cpp)
add_dependencies(simple_annotation_server yocs_simple_annotation_server)
target_link_libraries(simple_annotation_server yocs_simple_annotation_server)

install(TARGETS simple_annotation_server
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
