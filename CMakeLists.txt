cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROSPACK_MAKEDIST true)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
FIND_PACKAGE( OpenCV REQUIRED )

rosbuild_add_library(ar_track_alvar src/Camera.cpp src/Pose.cpp src/Marker.cpp src/MarkerDetector.cpp src/Bitset.cpp src/Rotation.cpp src/CvTestbed.cpp src/CaptureDevice.cpp src/CaptureFactory.cpp src/CaptureFactory_unix.cpp src/FileFormatUtils.cpp src/Threads.cpp src/Threads_unix.cpp src/Mutex.cpp src/Mutex_unix.cpp src/ConnectedComponents.cpp src/Line.cpp src/Plugin.cpp src/Plugin_unix.cpp src/DirectoryIterator.cpp src/DirectoryIterator_unix.cpp src/Draw.cpp src/Util.cpp src/Filter.cpp src/Kalman.cpp src/Optimization.cpp src/MultiMarker.cpp src/MultiMarkerBundle.cpp src/MultiMarkerInitializer.cpp)

TARGET_LINK_LIBRARIES(ar_track_alvar ${OpenCV_LIBS} )

# Kinect filtering code
rosbuild_add_library(kinect_filtering src/kinect_filtering.cpp)
rosbuild_add_library(medianFilter src/medianFilter.cpp)
rosbuild_add_compile_flags(kinect_filtering -std=c++0x)

set(ALVAR_TARGETS individualMarkers individualMarkersNoKinect trainMarkerBundle findMarkerBundles findMarkerBundlesNoKinect createMarker ar_track_alvar)

rosbuild_add_executable(individualMarkers nodes/IndividualMarkers.cpp)
target_link_libraries(individualMarkers ar_track_alvar kinect_filtering)

rosbuild_add_executable(individualMarkersNoKinect nodes/IndividualMarkersNoKinect.cpp)
target_link_libraries(individualMarkersNoKinect ar_track_alvar)

rosbuild_add_executable(trainMarkerBundle nodes/TrainMarkerBundle.cpp)
target_link_libraries(trainMarkerBundle ar_track_alvar)

rosbuild_add_executable(findMarkerBundles nodes/FindMarkerBundles.cpp)
target_link_libraries(findMarkerBundles ar_track_alvar kinect_filtering medianFilter)

rosbuild_add_executable(findMarkerBundlesNoKinect nodes/FindMarkerBundlesNoKinect.cpp)
target_link_libraries(findMarkerBundlesNoKinect ar_track_alvar)

rosbuild_add_executable(createMarker src/SampleMarkerCreator.cpp)
target_link_libraries(createMarker ar_track_alvar)

#rosbuild_add_executable(test_points test/test_points.cpp)
#target_link_libraries(test_points kinect_filtering)

#rosbuild_add_executable(test_kinect test/test_kinect_filtering.cpp)
#target_link_libraries(test_kinect kinect_filtering)


foreach(t ${ALVAR_TARGETS})
  rosbuild_add_compile_flags(${t}
    -Wno-unused-variable -Wno-unused-but-set-variable -Wno-missing-braces -Wno-comment -Wno-unused-value -Wno-sign-compare -Wno-comment -Wno-deprecated-declarations
    -Wno-type-limits)
endforeach(t)

rosbuild_make_distribution(0.5.0)

