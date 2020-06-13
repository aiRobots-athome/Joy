cmake_minimum_required(VERSION 3.5)
project(ScaraArm)
include_directories(${CMAKE_SOURCE_DIR})

add_library(ScaraArm SHARED ScaraArm.cpp) 
