cmake_minimum_required(VERSION 2.8.3)
project(incremental_calibration_examples_egomotion_sensors)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/../cmake/)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra")

cs_add_library(${PROJECT_NAME}
#  src/simulation/Trajectory.cpp
#  src/simulation/TrajectoryParams.cpp
#  src/simulation/SimulationData.cpp
#  src/simulation/SimulationParams.cpp
#  src/simulation/simulationEngine.cpp
  src/algo/bestQuat.cpp
  src/algo/OptimizationProblemSpline.cpp
  src/algo/CalibratorOptions.cpp
  src/algo/Calibrator.cpp
  src/algo/splinesToFile.cpp
)

find_package(Boost REQUIRED COMPONENTS system filesystem)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# Avoid clash with tr1::tuple:
# https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

#cs_add_executable(simulator src/simulation/simulator.cpp)
#target_link_libraries(simulator ${PROJECT_NAME})

cs_install()
cs_export()
