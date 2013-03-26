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

# find GSL library
find_package(PkgConfig REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)

rosbuild_add_library(${PROJECT_NAME}
  src/2dlrf/ErrorTermMotion.cpp
  src/2dlrf/ErrorTermObservation.cpp
  src/2dlrf/utils.cpp
  src/base/Serializable.cpp
  src/base/Timestamp.cpp
  src/base/Condition.cpp
  src/base/Mutex.cpp
  src/base/Thread.cpp
  src/base/Threads.cpp
  src/base/Timer.cpp
  src/exceptions/InvalidOperationException.cpp
  src/exceptions/SystemException.cpp
  src/statistics/NormalDistribution1v.cpp
  src/statistics/ChiSquareDistribution.cpp
  src/functions/DigammaFunction.cpp
  src/functions/IncompleteGammaPFunction.cpp
  src/functions/IncompleteGammaQFunction.cpp
  src/functions/LogFactorialFunction.cpp
  src/functions/LogGammaFunction.cpp
  src/car/ErrorTermPose.cpp
  src/car/ErrorTermOdometry.cpp
  src/vision/VisionDataAssociation.cpp
)
target_link_libraries(${PROJECT_NAME} ${gsl_LIBRARIES})

rosbuild_add_gtest(${PROJECT_NAME}_test
  test/test_main.cpp
  test/VectorDesignVariableTest.cpp
  test/ErrorTermMotionTest.cpp
  test/ErrorTermObservationTest.cpp
  test/ErrorTermPoseTest.cpp
  test/ErrorTermOdometryTest.cpp
)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

rosbuild_add_executable(2dlrf-simulate-batch src/2dlrf/simulate-batch.cpp)
target_link_libraries(2dlrf-simulate-batch ${PROJECT_NAME})

rosbuild_add_executable(2dlrf-simulate-it src/2dlrf/simulate-it.cpp)
target_link_libraries(2dlrf-simulate-it ${PROJECT_NAME})

rosbuild_add_executable(bsplineFromApplanix src/car/bsplineFromApplanix.cpp)
target_link_libraries(bsplineFromApplanix ${PROJECT_NAME})

rosbuild_add_executable(car-calibrate-batch-morsel
  src/car/calibrate-batch-morsel.cpp)
target_link_libraries(car-calibrate-batch-morsel ${PROJECT_NAME})

rosbuild_add_executable(applanix-optimize src/car/applanix-optimize.cpp)
target_link_libraries(applanix-optimize ${PROJECT_NAME})
