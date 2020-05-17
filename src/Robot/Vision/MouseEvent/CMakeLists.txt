cmake_minimum_required(VERSION 3.5)
project(MouseEvent)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(MouseEvent SHARED mouse_event.cpp)
target_include_directories(MouseEvent PUBLIC ${OpenCV_INCLUDE_DIRS})