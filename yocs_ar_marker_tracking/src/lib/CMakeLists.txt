###############################################################################
# Sources
###############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

###############################################################################
# Build
###############################################################################

add_library(yocs_ar_marker_tracking ${SOURCES})
add_dependencies(yocs_ar_marker_tracking ar_track_alvar_msgs_gencpp) # neccessary?
target_link_libraries(yocs_ar_marker_tracking yaml-cpp ${catkin_LIBRARIES})

###############################################################################
# Install
###############################################################################

install(TARGETS yocs_ar_marker_tracking
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
