cmake_minimum_required(VERSION 3.5)
project(XYPlatform)

add_library(XYPlatform XYPlatform.cpp)
target_link_libraries(XYPlatform MotorUnion)