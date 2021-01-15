cmake_minimum_required(VERSION 3.5)
project(PclConverter)

add_subdirectory(spec_msg)

find_package(PCL 1.10 REQUIRED COMPONENTS filters common PATHS /usr/lib/x86_64-linux-gnu/cmake/pcl)
find_package(catkin REQUIRED 
    COMPONENTS 
    nodelet
    geometry_msgs
    sensor_msgs
    pcl_conversions
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CATKIN_DEVEL_PREFIX}/include)

add_library(PclConverter SHARED pcl_converter.cpp)
add_dependencies(PclConverter spec_msg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(PclConverter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_include_directories(PclConverter PUBLIC ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX}/include)