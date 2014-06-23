cmake_minimum_required(VERSION 2.8.3)
project(ar_track_alvar_msgs)

set(MSG_DEPS
    std_msgs
    geometry_msgs
    )

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${MSG_DEPS}
)

set(MSG_FILES AlvarMarker.msg AlvarMarkers.msg)
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS message_runtime ${MSG_DEPS})
