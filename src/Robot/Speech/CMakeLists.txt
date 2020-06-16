cmake_minimum_required(VERSION 3.5)
project(Speech)

find_package(catkin REQUIRED COMPONENTS nodelet)
include_directories(${catkin_INCLUDE_DIRS})

add_library(Speech SHARED Speech.cpp)
target_link_libraries(Speech ${catkin_LIBRARIES})
target_include_directories(Speech PUBLIC ${catkin_INCLUDE_DIRS})