cmake_minimum_required(VERSION 3.5)
project(Vision)

add_subdirectory(ImageProcessing)
add_subdirectory(ImageConverter)
add_subdirectory(MouseEvent)
add_subdirectory(PclConverter)

add_library(Vision SHARED Vision.cpp)
target_link_libraries(Vision ImageProcessing ImageConverter MouseEvent PclConverter)