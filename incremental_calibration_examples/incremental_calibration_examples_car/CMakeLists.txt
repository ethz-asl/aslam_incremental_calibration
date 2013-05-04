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

# Find libposlv
find_path(LIBPOSLV_INCLUDE_DIR sensor/BinaryLogReader.h
  /usr/include
  PATH_SUFFIXES
  libposlv
)
message(Found ${LIBPOSLV_INCLUDE_DIR})
include_directories(${LIBPOSLV_INCLUDE_DIR})
find_library(LIBPOSLV_SENSOR poslv-sensor
  /usr/lib
)
find_library(LIBPOSLV_BASE poslv-base
  /usr/lib
)
find_library(LIBPOSLV_EXCEPTIONS poslv-exceptions
  /usr/lib
)
find_library(LIBPOSLV_GEO_TOOLS poslv-geo-tools
  /usr/lib
)

# libcan-prius
find_path(LIBCAN_PRIUS_INCLUDE_DIR sensor/PRIUSReader.h
  /usr/include
  PATH_SUFFIXES
  libcan-prius
)
message(Found ${LIBCAN_PRIUS_INCLUDE_DIR})
find_library(LIBCAN_PRIUS_SENSOR_LIBRARY can-prius-sensor
  /usr/lib
)
find_library(LIBCAN_PRIUS_TYPES_LIBRARY can-prius-types
  /usr/lib
)
include_directories(
  ${LIBCAN_PRIUS_INCLUDE_DIR}
)

rosbuild_add_library(${PROJECT_NAME}
  src/car/ErrorTermPose.cpp
  src/car/ErrorTermOdometry.cpp
  src/car/Odometry.cpp
  src/car/AckermanOdometry.cpp
  src/car/DifferentialOdometry.cpp
  src/car/ApplanixBinaryParser.cpp
  src/car/CANBinaryParser.cpp
  src/car/utils.cpp
)
target_link_libraries(${PROJECT_NAME} ${LIBPOSLV_SENSOR} ${LIBPOSLV_BASE}
  ${LIBPOSLV_EXCEPTIONS} ${LIBPOSLV_GEO_TOOLS} ${LIBCAN_PRIUS_SENSOR_LIBRARY}
  ${LIBCAN_PRIUS_TYPES_LIBRARY})

rosbuild_add_gtest(${PROJECT_NAME}_test
  test/test_main.cpp
  test/ErrorTermPoseTest.cpp
  test/ErrorTermOdometryTest.cpp
  test/OdometryTest.cpp
)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

rosbuild_add_executable(applanixBSpline src/car/applanixBSpline.cpp)
target_link_libraries(applanixBSpline ${PROJECT_NAME})

rosbuild_add_executable(applanixBSplineOptimize
  src/car/applanixBSplineOptimize.cpp)
target_link_libraries(applanixBSplineOptimize ${PROJECT_NAME})

rosbuild_add_executable(applanixPredOdometry
  src/car/applanixPredOdometry.cpp)
target_link_libraries(applanixPredOdometry ${PROJECT_NAME})

rosbuild_add_executable(odometryDump src/car/odometryDump.cpp)
target_link_libraries(odometryDump ${PROJECT_NAME})

rosbuild_add_executable(odometryBSpline src/car/odometryBSpline.cpp)
target_link_libraries(odometryBSpline ${PROJECT_NAME})

rosbuild_add_executable(car-calibrate-batch-morsel
  src/car/calibrate-batch-morsel.cpp)
target_link_libraries(car-calibrate-batch-morsel ${PROJECT_NAME})

rosbuild_add_executable(calibrate-prius-odometry-offline
  src/car/calibrate-prius-odometry-offline.cpp)
target_link_libraries(calibrate-prius-odometry-offline ${PROJECT_NAME})

rosbuild_add_executable(calibrate-prius-odometry-online
  src/car/calibrate-prius-odometry-online.cpp)
target_link_libraries(calibrate-prius-odometry-online ${PROJECT_NAME})
