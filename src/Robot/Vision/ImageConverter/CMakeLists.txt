cmake_minimum_required(VERSION 3.5)
project(ImageConverter)

find_package(catkin REQUIRED COMPONENTS image_transport cv_bridge nodelet)
include_directories(${catkin_INCLUDE_DIRS})

add_library(ImageConverter SHARED image_converter.cpp)
target_link_libraries(ImageConverter ${catkin_LIBRARIES})
target_include_directories(ImageConverter PUBLIC ${catkin_INCLUDE_DIRS})