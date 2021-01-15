cmake_minimum_required(VERSION 3.5)
project(Gui)

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

include_directories(${CMAKE_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

set(ui
ui/Form_Robot.ui 
ui/Form_Scara.ui
ui/Form_ControlPanel.ui 
ui/Form_May.ui 
ui/Form_Strategy.ui
)

set(headers
include/Robot/Form_Robot.h
include/Robot/Form_Arm.h
include/Robot/Form_Head.h
include/Robot/Form_Mobile.h
include/Robot/Form_ForceSensor.h
include/Scara/Form_Scara.h
include/Scara/Form_ScaraArm.h
include/Scara/Form_XYPlatform.h
include/Scara/Form_VisionCar.h
include/May/Form_May.h
include/May/Form_MayArm.h
include/Form_ControlPanel.h
include/Form_Strategy.h
)

set(src
src/Robot/Form_Robot.cpp
src/Robot/Form_Arm.cpp
src/Robot/Form_Head.cpp
src/Robot/Form_Mobile.cpp
src/Robot/Form_ForceSensor.cpp
src/Scara/Form_Scara.cpp
src/Scara/Form_ScaraArm.cpp
src/Scara/Form_XYPlatform.cpp
src/Scara/Form_VisionCar.cpp
src/May/Form_May.cpp
src/May/Form_MayArm.cpp
src/Form_ControlPanel.cpp
src/Form_Strategy.cpp
)

qt5_wrap_ui(ui_headers ${ui})
qt5_wrap_cpp(moc_headers ${headers})

add_library(Gui SHARED ${ui_headers} ${src} ${moc_headers})
target_link_libraries(Gui Qt5::Widgets Qt5::Core Robot XBoxJoystick Scara)
target_include_directories(Gui PUBLIC ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)