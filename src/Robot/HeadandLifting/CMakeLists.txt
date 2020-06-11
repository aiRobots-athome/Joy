cmake_minimum_required(VERSION 3.5)
project(HeadandLifting)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(HeadandLifting SHARED HeadandLifting.cpp)
target_link_libraries(HeadandLifting MotorUnion ${OpenCV_LIBS})