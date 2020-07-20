cmake_minimum_required(VERSION 3.5)
project(Scara)

add_subdirectory(ScaraArm)
add_subdirectory(XYPlatform)
add_subdirectory(VisionCar)

add_library(Scara SHARED Scara.cpp)
target_link_libraries(Scara ScaraArm XYPlatform VisionCar)