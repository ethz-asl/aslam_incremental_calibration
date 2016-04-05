cmake_minimum_required(VERSION 2.8.3)
project(aslam_tsvd_solver)

find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions("-std=c++11")

cs_add_library(${PROJECT_NAME}
  src/aslam-tsvd-solver.cc
)
target_link_libraries(${PROJECT_NAME})

cs_install()
cs_export()
