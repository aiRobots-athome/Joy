cmake_minimum_required(VERSION 3.5)
project(Body)

add_subdirectory(Arm)
add_subdirectory(HeadandLiftingPlatform)
add_subdirectory(MobilePlatform)
add_subdirectory(MotorUnion)

add_library(Body SHARED Body.cpp)
target_link_libraries(Body Arm HeadandLiftingPlatform MobilePlatform MotorUnion)

# add_executable(Body_project main.cpp)
# target_link_libraries(Body_project Body)