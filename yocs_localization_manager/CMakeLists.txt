cmake_minimum_required(VERSION 2.8.3)
project(yocs_localization_manager)

## Find catkin macros and libraries
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslint
)

catkin_python_setup()

catkin_package()

install(
  PROGRAMS
    scripts/localization_manager.py
    scripts/tracker_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Lint Python modules for PEP8 compatibility
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SCRIPTS
     RELATIVE ${PROJECT_SOURCE_DIR} scripts/*)
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRC
RELATIVE ${PROJECT_SOURCE_DIR} src/${PROJECT_NAME}/*.py)
roslint_python(${${PROJECT_NAME}_PY_SCRIPTS})
roslint_python(${${PROJECT_NAME}_PY_SRC})
