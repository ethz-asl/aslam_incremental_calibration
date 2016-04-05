cmake_minimum_required(VERSION 2.8.3)
project(truncated_svd_solver)

find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions("-std=c++11")

cs_add_library(${PROJECT_NAME}
  src/linear-algebra-helpers.cc
  src/marginalization.cc
  src/timing.cc
  src/tsvd-solver.cc
  src/tsvd-solver-options.cc
)
target_link_libraries(${PROJECT_NAME} pthread ${TBB_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}_test
  test/test_main.cpp
  test/test-tsvd-solver.cc
)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_install()
cs_export()
