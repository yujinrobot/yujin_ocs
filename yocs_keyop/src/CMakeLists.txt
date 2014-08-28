##############################################################################
# Sources
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# Targets
##############################################################################

add_executable(yocs_keyop ${SOURCES})
target_link_libraries(yocs_keyop ${catkin_LIBRARIES})

install(TARGETS yocs_keyop
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
