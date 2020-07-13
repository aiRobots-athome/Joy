cmake_minimum_required(VERSION 3.5)
project(ScaraArm)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(ScaraArm SHARED ScaraArm.cpp)
target_link_libraries(ScaraArm ${OpenCV_LIBRARIES}) 
target_include_directories(ScaraArm PUBLIC ${OpenCV_INCLUDE_DIRS})