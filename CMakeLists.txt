cmake_minimum_required(VERSION 2.8.3)
project(ar_track_alvar)

set(MSG_DEPS
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs)

find_package(catkin COMPONENTS
        genmsg
        roscpp
        tf
        image_transport
        resource_retriever
        cv_bridge
        pcl_ros
        message_generation
        ${MSG_DEPS}
        REQUIRED)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig)
pkg_check_modules(PC_TINYXML REQUIRED tinyxml)

# generate messages
set(MSG_FILES AlvarMarker.msg AlvarMarkers.msg)
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ar_track_alvar
  CATKIN_DEPENDS
        std_msgs
        roscpp
        tf
        message_runtime
        image_transport
        sensor_msgs
        geometry_msgs
        visualization_msgs
        resource_retriever
        cv_bridge
        pcl_ros
)

include_directories(include 
                    ${PC_TINYXML_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${PC_TINYXML_INCLUDE_DIRS}

)

set(GENCPP_DEPS std_msgs_gencpp sensor_msgs_gencpp geometry_msgs_gencpp visualization_msgs_gencpp)

add_library(ar_track_alvar
    src/Camera.cpp
    src/CaptureDevice.cpp
    src/Pose.cpp
    src/Marker.cpp
    src/MarkerDetector.cpp
    src/Bitset.cpp
    src/Rotation.cpp
    src/CvTestbed.cpp
    src/CaptureDevice.cpp
    src/CaptureFactory.cpp
    src/CaptureFactory_unix.cpp
    src/FileFormatUtils.cpp
    src/Threads.cpp
    src/Threads_unix.cpp
    src/Mutex.cpp
    src/Mutex_unix.cpp
    src/ConnectedComponents.cpp
    src/Line.cpp src/Plugin.cpp
    src/Plugin_unix.cpp
    src/DirectoryIterator.cpp
    src/DirectoryIterator_unix.cpp
    src/Draw.cpp
    src/Util.cpp
    src/Filter.cpp
    src/Kalman.cpp
    src/kinect_filtering.cpp
    src/Optimization.cpp
    src/MultiMarker.cpp
    src/MultiMarkerBundle.cpp
    src/MultiMarkerInitializer.cpp)
target_link_libraries(ar_track_alvar ${OpenCV_LIBS} tinyxml ${catkin_LIBRARIES})
add_dependencies(ar_track_alvar ${GENCPP_DEPS})

# Kinect filtering code
add_library(kinect_filtering src/kinect_filtering.cpp)
target_link_libraries(kinect_filtering ${catkin_LIBRARIES})
add_dependencies(kinect_filtering ${GENCPP_DEPS})

add_library(medianFilter src/medianFilter.cpp)
target_link_libraries(medianFilter ${catkin_LIBRARIES})
add_dependencies(medianFilter ${GENCPP_DEPS})

set(ALVAR_TARGETS ar_track_alvar individualMarkers individualMarkersNoKinect trainMarkerBundle findMarkerBundles findMarkerBundlesNoKinect createMarker ar_track_alvar)

add_executable(individualMarkers nodes/IndividualMarkers.cpp)
target_link_libraries(individualMarkers ar_track_alvar kinect_filtering ${catkin_LIBRARIES})
add_dependencies(individualMarkers ${PROJECT_NAME}_gencpp ${GENCPP_DEPS})

add_executable(individualMarkersNoKinect nodes/IndividualMarkersNoKinect.cpp)
target_link_libraries(individualMarkersNoKinect ar_track_alvar ${catkin_LIBRARIES})
add_dependencies(individualMarkersNoKinect ${PROJECT_NAME}_gencpp  ${GENCPP_DEPS})

add_executable(trainMarkerBundle nodes/TrainMarkerBundle.cpp)
target_link_libraries(trainMarkerBundle ar_track_alvar ${catkin_LIBRARIES})
add_dependencies(trainMarkerBundle ${PROJECT_NAME}_gencpp ${GENCPP_DEPS})

add_executable(findMarkerBundles nodes/FindMarkerBundles.cpp)
target_link_libraries(findMarkerBundles ar_track_alvar kinect_filtering medianFilter ${catkin_LIBRARIES})
add_dependencies(findMarkerBundles ${PROJECT_NAME}_gencpp ${GENCPP_DEPS})

add_executable(findMarkerBundlesNoKinect nodes/FindMarkerBundlesNoKinect.cpp)
target_link_libraries(findMarkerBundlesNoKinect ar_track_alvar ${catkin_LIBRARIES})
add_dependencies(findMarkerBundlesNoKinect ${PROJECT_NAME}_gencpp ${GENCPP_DEPS})

add_executable(createMarker src/SampleMarkerCreator.cpp)
target_link_libraries(createMarker ar_track_alvar ${catkin_LIBRARIES})
add_dependencies(createMarker ${PROJECT_NAME}_gencpp ${GENCPP_DEPS})

install(TARGETS ${ALVAR_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
