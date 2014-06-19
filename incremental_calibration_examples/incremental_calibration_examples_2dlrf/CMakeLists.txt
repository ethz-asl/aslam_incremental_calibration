cmake_minimum_required(VERSION 2.8.3)
project(incremental_calibration_examples_2dlrf)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/../cmake/)

find_package(catkin_simple REQUIRED)
catkin_simple()

if(APPLE)
  set(CMAKE_CXX_FLAGS "-std=c++11")
else()
  set(CMAKE_CXX_FLAGS "-std=c++0x")
endif()

cs_add_library(${PROJECT_NAME}
  src/2dlrf/ErrorTermMotion.cpp
  src/2dlrf/ErrorTermObservation.cpp
  src/2dlrf/utils.cpp
)

find_package(Boost REQUIRED COMPONENTS system filesystem)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# Avoid clash with tr1::tuple:
# https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

catkin_add_gtest(${PROJECT_NAME}_test
  test/test_main.cpp
  test/ErrorTermMotionTest.cpp
  test/ErrorTermObservationTest.cpp
)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_add_executable(2dlrf-simulate-offline src/2dlrf/simulate-offline.cpp)
target_link_libraries(2dlrf-simulate-offline ${PROJECT_NAME})

cs_add_executable(2dlrf-simulate-online src/2dlrf/simulate-online.cpp)
target_link_libraries(2dlrf-simulate-online ${PROJECT_NAME})

cs_add_executable(2dlrf-simulate-online-new
  src/2dlrf/simulate-online-new.cpp)
target_link_libraries(2dlrf-simulate-online-new ${PROJECT_NAME})

cs_install()
cs_export()
