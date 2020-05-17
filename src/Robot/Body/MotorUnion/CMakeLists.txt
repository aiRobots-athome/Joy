cmake_minimum_required(VERSION 3.5)
project(MotorUnion)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(Motor)

add_library(MotorUnion SHARED MotorUnion.cpp)
target_link_libraries(MotorUnion Motor)

add_executable(a main.cpp)
target_link_libraries(a MotorUnion)