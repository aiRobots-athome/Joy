cmake_minimum_required(VERSION 3.5)
project(ImageProcessing)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(ImageProcessing SHARED ImageProcessing.cpp)
target_link_libraries(ImageProcessing ${OpenCV_LIBRARIES})
target_include_directories(ImageProcessing PUBLIC ${OpenCV_INCLUDE_DIRS})