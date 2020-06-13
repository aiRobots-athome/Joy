cmake_minimum_required(VERSION 3.5)
project(XY_Platform)

add_library(XY_Platform XY_Platform.cpp)
target_link_libraries(XY_Platform MotorUnion)