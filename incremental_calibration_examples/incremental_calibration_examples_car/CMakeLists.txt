cmake_minimum_required(VERSION 2.8.3)
project(incremental_calibration_examples_car)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/../cmake/)

find_package(catkin_simple REQUIRED)
catkin_simple()

if(APPLE)
  set(CMAKE_CXX_FLAGS "-std=c++11")
else()
  set(CMAKE_CXX_FLAGS "-std=c++0x")
endif()

cs_add_library(${PROJECT_NAME}
  src/error-terms/ErrorTermWheel.cpp
  src/error-terms/ErrorTermSteering.cpp
  src/error-terms/ErrorTermPose.cpp
  src/error-terms/ErrorTermVelocities.cpp
  src/algo/CarCalibratorOptions.cpp
  src/algo/OptimizationProblemSpline.cpp
  src/algo/CarCalibrator.cpp
  src/algo/bestQuat.cpp
  src/algo/splinesToFile.cpp
  src/design-variables/OdometryDesignVariables.cpp
  src/geo/geodetic.cpp
)

find_package(Boost REQUIRED COMPONENTS system filesystem)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# Avoid clash with tr1::tuple:
# https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

catkin_add_gtest(${PROJECT_NAME}_test
  test/test_main.cpp
  test/error-terms/ErrorTermWheelTest.cpp
  test/error-terms/ErrorTermSteeringTest.cpp
  test/error-terms/ErrorTermPoseTest.cpp
  test/error-terms/ErrorTermVelocitiesTest.cpp
)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_add_executable(calibrator src/realworld/calibrator.cpp)
target_link_libraries(calibrator ${PROJECT_NAME})

cs_add_executable(analyzer src/realworld/analyzer.cpp)
target_link_libraries(analyzer ${PROJECT_NAME})

cs_install()
cs_export()
