cmake_minimum_required(VERSION 2.8.3)
project(incremental_calibration_examples_camera)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/../cmake/)

find_package(catkin_simple REQUIRED)
catkin_simple()

if(APPLE)
  set(CMAKE_CXX_FLAGS "-std=c++11")
else()
  set(CMAKE_CXX_FLAGS "-std=c++0x")
endif()

cs_add_library(${PROJECT_NAME}
  src/camera/CameraCalibrator.cpp
  src/camera/CameraValidator.cpp
)

#find_package(Boost REQUIRED COMPONENTS system filesystem)
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# Avoid clash with tr1::tuple:
# https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

cs_add_executable(calibrateCamera src/camera/calibrateCamera.cpp)
target_link_libraries(calibrateCamera ${PROJECT_NAME})

cs_add_executable(validateCamera src/camera/validateCamera.cpp)
target_link_libraries(validateCamera ${PROJECT_NAME})

cs_install()
cs_export()
