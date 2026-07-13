cmake_minimum_required(VERSION 3.10)

project(sp_vision_25_main)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

# 海康 MVS SDK 路径，机器人上如果不是 /opt/MVS，就改这里
set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK root path")

find_path(MVS_INCLUDE_DIR
    NAMES MvCameraControl.h
    PATHS ${MVS_ROOT}/include
    REQUIRED
)

find_library(MVS_CAMERA_LIB
    NAMES MvCameraControl
    PATHS ${MVS_ROOT}/lib/64
    REQUIRED
)

add_executable(yolo26_hik_usb_pose
    src/yolo26_hik_usb_pose.cpp
)

target_include_directories(yolo26_hik_usb_pose PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${MVS_INCLUDE_DIR}
)

target_link_libraries(yolo26_hik_usb_pose PRIVATE
    ${OpenCV_LIBS}
    ${MVS_CAMERA_LIB}
)

# 让运行时能找到 libMvCameraControl.so
set_target_properties(yolo26_hik_usb_pose PROPERTIES
    BUILD_RPATH "${MVS_ROOT}/lib/64"
)