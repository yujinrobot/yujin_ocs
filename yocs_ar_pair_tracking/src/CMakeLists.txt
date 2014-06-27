###############################################################################
# Subdirectories
###############################################################################

add_subdirectory(lib)
add_subdirectory(client)

###############################################################################
# Nodes
###############################################################################

add_executable(yocs_ar_pair_tracking_node main.cpp)
target_link_libraries(yocs_ar_pair_tracking_node yocs_ar_pair_tracking)

###############################################################################
# Install
###############################################################################

install(TARGETS yocs_ar_pair_tracking_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
