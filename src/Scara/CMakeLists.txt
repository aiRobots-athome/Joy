cmake_minimum_required(VERSION 3.5)
project(Scara)
include_directories(${CMAKE_SOURCE_DIR})

add_subdirectory(ScaraArm)

add_library(Scara SHARED Scara.cpp)
target_link_libraries(Scara ScaraArm)

