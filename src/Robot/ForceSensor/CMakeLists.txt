cmake_minimum_required(VERSION 2.8.3)

project(ForceSensor)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

set(ROS_INCLUDE_DIRS /opt/ros/noetic/include)
include_directories(${ROS_INCLUDE_DIRS})

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

add_library(ForceSensor ForceSensor.cpp)
target_link_libraries(ForceSensor ${catkin_LIBRARIES})