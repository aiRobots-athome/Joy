cmake_minimum_required(VERSION 3.5)
project(Mobile)

add_subdirectory(Steering)
add_subdirectory(Wheel)

add_library(Mobile SHARED Mobile.cpp)
target_link_libraries(Mobile MotorUnion Steering Wheel)
