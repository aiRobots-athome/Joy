cmake_minimum_required(VERSION 3.5)
project(Motor)

add_subdirectory(dynamixel)
add_subdirectory(motor)

add_library(Motor SHARED Motor.cpp MotorMx.cpp MotorPro.cpp MotorProPlus.cpp)
target_link_libraries(Motor DynamixelSdk motor)