cmake_minimum_required(VERSION 3.5)
project(ScaraArm)

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#link_directories(${EIGEN3_INCLUDE_DIR})

add_library(ScaraArm SHARED ScaraArm.cpp)
# target_link_libraries(ScaraArm Eigen)