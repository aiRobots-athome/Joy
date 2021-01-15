cmake_minimum_required(VERSION 3.5)
project(May)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(May SHARED May.cpp)
target_link_libraries(May MotorUnion ${OpenCV_LIBS})
target_include_directories(May PUBLIC ${OpenCV_INCLUDE_DIRS})