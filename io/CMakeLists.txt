cmake_minimum_required(VERSION 3.16)

# 查找依赖
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# 串口库
add_subdirectory(serial)

# 创建 io 静态库
add_library(io STATIC
    hikrobot/hikrobot.cpp
    mindvision/mindvision.cpp
    usbcamera/usbcamera.cpp
    camera.cpp
    cboard.cpp
    dm_imu/dm_imu.cpp
    gimbal/gimbal.cpp
)

# io 模块需要的头文件路径
target_include_directories(io PUBLIC
    # 项目根目录
    ${PROJECT_SOURCE_DIR}

    # OpenCV，例如 /usr/include/opencv4
    ${OpenCV_INCLUDE_DIRS}

    # 海康相机 SDK
    ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/include

    # 迈德威视相机 SDK
    ${CMAKE_CURRENT_SOURCE_DIR}/mindvision/include
)

################## 海康相机 SDK ##################

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    target_link_directories(io PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/amd64
    )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    target_link_directories(io PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/arm64
    )
else()
    message(
        FATAL_ERROR
        "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}"
    )
endif()

################## 迈德威视相机 SDK ##################

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    target_link_directories(io PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/mindvision/lib/amd64
    )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    target_link_directories(io PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/mindvision/lib/arm64
    )
else()
    message(
        FATAL_ERROR
        "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}"
    )
endif()

################## 链接依赖库 ##################

target_link_libraries(io PUBLIC
    MvCameraControl
    MVSDK
    usb-1.0
    yaml-cpp
    serial
    Threads::Threads
    ${OpenCV_LIBS}
)

################## 检查 ROS2 环境 ##################

find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)

# ROS2 依赖完整时才编译 ROS2 代码
if(
    NOT ament_cmake_FOUND
    OR NOT rclcpp_FOUND
    OR NOT std_msgs_FOUND
    OR NOT rosidl_typesupport_cpp_FOUND
    OR NOT sp_msgs_FOUND
)
    message(
        WARNING
        "ROS2 not found, skipping ROS2 specific code."
    )
else()
    message(
        STATUS
        "ROS2 environment found, compiling ROS2-related code."
    )

    target_sources(io PRIVATE
        ros2/publish2nav.cpp
        ros2/ros2.cpp
        ros2/subscribe2nav.cpp
    )

    ament_target_dependencies(io
        rclcpp
        std_msgs
        sp_msgs
    )
endif()
