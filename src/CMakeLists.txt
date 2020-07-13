cmake_minimum_required(VERSION 3.5)
project(Joy)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(Gui)
# add_subdirectory(Network)
add_subdirectory(Robot)
add_subdirectory(Scara)
add_subdirectory(Strategy)
add_subdirectory(XBoxJoystick)

add_executable(Joy main.cpp)
target_link_libraries(Joy 
  Gui 
  Robot
  Scara
  Strategy 
  XBoxJoystick
)
