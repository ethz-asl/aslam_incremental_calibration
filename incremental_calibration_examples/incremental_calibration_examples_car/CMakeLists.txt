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

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Package config
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

# Find libposlv
pkg_check_modules(LIBPOSLV REQUIRED libposlv)
include_directories(${LIBPOSLV_INCLUDE_DIRS})
link_directories(${LIBPOSLV_LIBRARY_DIRS})
pkg_check_modules(LIBPOSLV_GEO_TOOLS REQUIRED libposlv-geo-tools)
include_directories(${LIBPOSLV_GEO_TOOLS_INCLUDE_DIRS})
link_directories(${LIBPOSLV_GEO_TOOLS_LIBRARY_DIRS})

# Find libcan-prius
pkg_check_modules(LIBCAN_PRIUS REQUIRED libcan-prius)
include_directories(${LIBCAN_PRIUS_INCLUDE_DIRS})
link_directories(${LIBCAN_PRIUS_LIBRARY_DIRS})

# Find robot-odometry
pkg_check_modules(ROBOT_ODOMETRY REQUIRED robot-odometry)
include_directories(${ROBOT_ODOMETRY_INCLUDE_DIRS})
link_directories(${ROBOT_ODOMETRY_LIBRARY_DIRS})
pkg_check_modules(ROBOT_ODOMETRY REQUIRED robot-odometry-janeth)
include_directories(${ROBOT_ODOMETRY_JANETH_INCLUDE_DIRS})
link_directories(${ROBOT_ODOMETRY_JANETH_LIBRARY_DIRS})

rosbuild_add_library(${PROJECT_NAME}
  src/car/ErrorTermPose.cpp
  src/car/ErrorTermOdometry.cpp
  src/car/utils.cpp
  src/car/ErrorTermFws.cpp
  src/car/ErrorTermRws.cpp
  src/car/ErrorTermSteering.cpp
  src/car/ErrorTermDMI.cpp
  src/car/CarCalibrator.cpp
  src/car/OptimizationProblemSpline.cpp
  src/car/CovarianceEstimator1v.cpp
)

rosbuild_add_gtest(${PROJECT_NAME}_test
  test/test_main.cpp
  test/ErrorTermPoseTest.cpp
  test/ErrorTermOdometryTest.cpp
  test/CarCalibratorTest.cpp
  test/CovarianceEstimatorTest.cpp
)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

rosbuild_add_executable(calibrateCarOdometryMorselBatch
  src/car/calibrateCarOdometryMorselBatch.cpp)
target_link_libraries(calibrateCarOdometryMorselBatch ${PROJECT_NAME})

rosbuild_add_executable(processBagNewSpline src/car/processBagNewSpline.cpp)
target_link_libraries(processBagNewSpline ${PROJECT_NAME}
  ${LIBPOSLV_GEO_TOOLS_LIBRARIES})

rosbuild_add_executable(processBagOldSpline src/car/processBagOldSpline.cpp)
target_link_libraries(processBagOldSpline ${PROJECT_NAME}
  ${LIBPOSLV_GEO_TOOLS_LIBRARIES})

rosbuild_add_executable(calibrateCar src/car/calibrateCar.cpp)
target_link_libraries(calibrateCar ${PROJECT_NAME}
  ${LIBPOSLV_GEO_TOOLS_LIBRARIES})

rosbuild_add_executable(optimizeNewSpline src/car/optimizeNewSpline.cpp)
target_link_libraries(optimizeNewSpline ${PROJECT_NAME}
  ${LIBPOSLV_GEO_TOOLS_LIBRARIES})

rosbuild_add_executable(optimizeOldSpline src/car/optimizeOldSpline.cpp)
target_link_libraries(optimizeOldSpline ${PROJECT_NAME}
  ${LIBPOSLV_GEO_TOOLS_LIBRARIES})

rosbuild_add_executable(calibrateCarOdometryNewSplineBatch
  src/car/calibrateCarOdometryNewSplineBatch.cpp)
target_link_libraries(calibrateCarOdometryNewSplineBatch ${PROJECT_NAME}
  ${LIBPOSLV_GEO_TOOLS_LIBRARIES} ${ROBOT_ODOMETRY_LIBRARIES})

rosbuild_add_executable(calibrateCarOdometryOldSplineBatch
  src/car/calibrateCarOdometryOldSplineBatch.cpp)
target_link_libraries(calibrateCarOdometryOldSplineBatch ${PROJECT_NAME}
  ${LIBPOSLV_GEO_TOOLS_LIBRARIES})

rosbuild_add_executable(convertBinaryToBag src/car/convertBinaryToBag.cpp)
target_link_libraries(convertBinaryToBag ${PROJECT_NAME}
  ${LIBPOSLV_LIBRARIES} ${LIBCAN_PRIUS_LIBRARIES})

rosbuild_add_executable(simulateCarOdometry src/car/simulateCarOdometry.cpp)
target_link_libraries(simulateCarOdometry ${PROJECT_NAME}
  ${LIBPOSLV_GEO_TOOLS_LIBRARIES})

rosbuild_add_executable(calibrateCarOdometrySimulationBatch
  src/car/calibrateCarOdometrySimulationBatch.cpp)
target_link_libraries(calibrateCarOdometrySimulationBatch ${PROJECT_NAME}
  ${LIBPOSLV_GEO_TOOLS_LIBRARIES} ${ROBOT_ODOMETRY_LIBRARIES}
  ${ROBOT_ODOMETRY_JANETH_LIBRARIES})

rosbuild_add_executable(calibrateCarOdometrySimulationBatchTest
  src/car/calibrateCarOdometrySimulationBatchTest.cpp)
target_link_libraries(calibrateCarOdometrySimulationBatchTest ${PROJECT_NAME}
  ${LIBPOSLV_GEO_TOOLS_LIBRARIES} ${ROBOT_ODOMETRY_LIBRARIES}
  ${ROBOT_ODOMETRY_JANETH_LIBRARIES})
