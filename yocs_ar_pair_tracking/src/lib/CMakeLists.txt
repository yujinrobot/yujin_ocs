###############################################################################
# Sources
###############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

###############################################################################
# Build
###############################################################################

add_library(yocs_ar_pair_tracking ${SOURCES})
target_link_libraries(yocs_ar_pair_tracking yaml-cpp ${catkin_LIBRARIES})

###############################################################################
# Install
###############################################################################

install(TARGETS yocs_ar_pair_tracking
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
