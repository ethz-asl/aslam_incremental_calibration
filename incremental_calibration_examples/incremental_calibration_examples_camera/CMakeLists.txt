cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

if(APPLE)
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wno-unused-local-typedefs")
else()
  set(CMAKE_CXX_FLAGS "-std=c++0x")
endif()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME}
  src/camera/CameraCalibrator.cpp
)

#rosbuild_add_gtest(${PROJECT_NAME}_test
#  test/test_main.cpp
#  test/ErrorTermMotionTest.cpp
#  test/ErrorTermObservationTest.cpp
#)
#target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

#rosbuild_add_executable(2dlrf-simulate-offline src/2dlrf/simulate-offline.cpp)
#target_link_libraries(2dlrf-simulate-offline ${PROJECT_NAME})

#rosbuild_add_executable(2dlrf-simulate-online src/2dlrf/simulate-online.cpp)
#target_link_libraries(2dlrf-simulate-online ${PROJECT_NAME})

#rosbuild_add_executable(2dlrf-simulate-online-new
#  src/2dlrf/simulate-online-new.cpp)
#target_link_libraries(2dlrf-simulate-online-new ${PROJECT_NAME})
