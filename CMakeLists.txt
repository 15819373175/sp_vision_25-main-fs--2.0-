cmake_minimum_required(VERSION 3.16)

project(sp_vision_25_main LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==========================================
# 全局查找依赖
# ==========================================
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# ── fmt / spdlog ──
# logger.cpp、recorder.cpp、部分测试程序会直接使用 fmt/spdlog。
# 必须同时链接 fmt 和 spdlog，否则会出现 fmt::v8 undefined reference。
find_package(fmt CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

# ── 海康 MVS SDK ──
set(MVS_ROOT "/opt/MVS"
    CACHE PATH "Hikrobot MVS SDK root path"
)

find_path(MVS_INCLUDE_DIR
    NAMES MvCameraControl.h
    PATHS
        ${MVS_ROOT}/include
    REQUIRED
)

find_library(MVS_CAMERA_LIB
    NAMES MvCameraControl
    PATHS
        ${MVS_ROOT}/lib/64
        ${MVS_ROOT}/lib
    REQUIRED
)

# ── OpenVINO ──
set(OpenVINO_DIR
    "/opt/intel/openvino_2024.6.0/runtime/cmake"
    CACHE PATH "OpenVINO CMake config directory"
)

find_package(OpenVINO QUIET)

if(NOT OpenVINO_FOUND)
    message(WARNING
        "OpenVINO not found — auto_aim / auto_buff will NOT build"
    )
endif()

# ── Ceres（打符需要） ──
find_package(Ceres QUIET)

if(NOT Ceres_FOUND)
    message(WARNING
        "Ceres not found — auto_buff will NOT build"
    )
endif()

# ==========================================
# 公共 include
# ==========================================
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${MVS_INCLUDE_DIR}
)

# ==========================================
# 子目录
# 顺序敏感
# ==========================================
add_subdirectory(tools)

# tools/logger.cpp 和 tools/recorder.cpp 使用了 fmt/spdlog。
# 设置为 PUBLIC 后，所有链接 tools 的程序都会自动继承这些库。
if(NOT TARGET tools)
    message(FATAL_ERROR
        "tools/CMakeLists.txt 没有创建名为 tools 的 CMake target"
    )
endif()

target_link_libraries(tools
    PUBLIC
        fmt::fmt
        spdlog::spdlog
        Threads::Threads
)

add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)

# ==========================================
# 公共可执行目标创建函数
# ==========================================
function(add_sp_target TARGET_NAME SOURCE_FILE)
    add_executable(
        ${TARGET_NAME}
        ${SOURCE_FILE}
    )

    target_link_libraries(
        ${TARGET_NAME}
        PRIVATE
            io
            serial
            tools
            auto_aim
            tinympcstatic
            ${OpenCV_LIBS}
            ${MVS_CAMERA_LIB}
            yaml-cpp
    )

    if(OpenVINO_FOUND)
        target_link_libraries(
            ${TARGET_NAME}
            PRIVATE
                openvino::runtime
        )
    endif()
endfunction()

# ==========================================
# 主可执行文件
# ==========================================
add_sp_target(
    standard_mpc
    src/standard_mpc.cpp
)
# standard_mpc 使用大符检测、解算和预测模块
target_link_libraries(
    standard_mpc
    PRIVATE
        auto_buff
)
add_sp_target(
    standard
    src/standard.cpp
)

add_sp_target(
    auto_aim_debug_mpc
    src/auto_aim_debug_mpc.cpp
)

# ==========================================
# 多线程主程序
# ==========================================

# ── mt_standard ──
add_executable(
    mt_standard
    src/mt_standard.cpp
)

target_link_libraries(
    mt_standard
    PRIVATE
        io
        serial
        tools
        auto_aim
        auto_buff
        tinympcstatic
        ${OpenCV_LIBS}
        ${MVS_CAMERA_LIB}
        yaml-cpp
)

if(OpenVINO_FOUND)
    target_link_libraries(
        mt_standard
        PRIVATE
            openvino::runtime
    )
endif()

if(Ceres_FOUND)
    target_link_libraries(
        mt_standard
        PRIVATE
            ${CERES_LIBRARIES}
    )
endif()

# ── mt_auto_aim_debug ──
add_executable(
    mt_auto_aim_debug
    src/mt_auto_aim_debug.cpp
)

target_link_libraries(
    mt_auto_aim_debug
    PRIVATE
        io
        serial
        tools
        auto_aim
        auto_buff
        tinympcstatic
        ${OpenCV_LIBS}
        ${MVS_CAMERA_LIB}
        yaml-cpp
)

if(OpenVINO_FOUND)
    target_link_libraries(
        mt_auto_aim_debug
        PRIVATE
            openvino::runtime
    )
endif()

if(Ceres_FOUND)
    target_link_libraries(
        mt_auto_aim_debug
        PRIVATE
            ${CERES_LIBRARIES}
    )
endif()

# ==========================================
# 打符程序
# ==========================================
add_executable(
    auto_buff_debug
    src/auto_buff_debug.cpp
)

add_executable(
    auto_buff_debug_mpc
    src/auto_buff_debug_mpc.cpp
)

foreach(buff_target
    auto_buff_debug
    auto_buff_debug_mpc
)
    target_link_libraries(
        ${buff_target}
        PRIVATE
            io
            serial
            tools
            auto_aim
            auto_buff
            tinympcstatic
            ${OpenCV_LIBS}
            ${MVS_CAMERA_LIB}
            yaml-cpp
    )

    if(OpenVINO_FOUND)
        target_link_libraries(
            ${buff_target}
            PRIVATE
                openvino::runtime
        )
    endif()

    if(Ceres_FOUND)
        target_link_libraries(
            ${buff_target}
            PRIVATE
                ${CERES_LIBRARIES}
        )
    endif()
endforeach()

# ==========================================
# UAV
# ==========================================
add_sp_target(
    uav
    src/uav.cpp
)
# uav.cpp 使用打符模块
target_link_libraries(
    uav
    PRIVATE
        auto_buff
)
add_sp_target(
    uav_debug
    src/uav_debug.cpp
)

# ==========================================
# 测试程序
# ==========================================

# ── 云台测试 ──
add_executable(
    gimbal_test
    tests/gimbal_test.cpp
)

add_executable(
    gimbal_response_test
    tests/gimbal_response_test.cpp
)

add_executable(
    cboard_test
    tests/cboard_test.cpp
)

foreach(test_target
    gimbal_test
    gimbal_response_test
    cboard_test
)
    target_link_libraries(
        ${test_target}
        PRIVATE
            io
            serial
            tools
            ${OpenCV_LIBS}
            ${MVS_CAMERA_LIB}
            yaml-cpp
    )
endforeach()

# ── 相机测试 ──
add_executable(
    camera_test
    tests/camera_test.cpp
)

add_executable(
    camera_thread_test
    tests/camera_thread_test.cpp
)

add_executable(
    usbcamera_test
    tests/usbcamera_test.cpp
)

foreach(test_target
    camera_test
    camera_thread_test
    usbcamera_test
)
    target_link_libraries(
        ${test_target}
        PRIVATE
            io
            serial
            tools
            ${OpenCV_LIBS}
            ${MVS_CAMERA_LIB}
            yaml-cpp
    )
endforeach()
# camera_thread_test 使用 auto_aim::YOLO
target_link_libraries(
    camera_thread_test
    PRIVATE
        auto_aim
)
# ── 自瞄测试 ──
add_executable(
    auto_aim_test
    tests/auto_aim_test.cpp
)

target_link_libraries(
    auto_aim_test
    PRIVATE
        io
        serial
        tools
        auto_aim
        tinympcstatic
        ${OpenCV_LIBS}
        ${MVS_CAMERA_LIB}
        yaml-cpp
)

if(OpenVINO_FOUND)
    target_link_libraries(
        auto_aim_test
        PRIVATE
            openvino::runtime
    )
endif()

# ── Planner 测试 ──
add_executable(
    planner_test
    tests/planner_test.cpp
)

add_executable(
    planner_test_offline
    tests/planner_test_offline.cpp
)

foreach(test_target
    planner_test
    planner_test_offline
)
    target_link_libraries(
        ${test_target}
        PRIVATE
            io
            serial
            tools
            auto_aim
            tinympcstatic
            ${OpenCV_LIBS}
            ${MVS_CAMERA_LIB}
            yaml-cpp
    )

    if(OpenVINO_FOUND)
        target_link_libraries(
            ${test_target}
            PRIVATE
                openvino::runtime
        )
    endif()
endforeach()

# ── 打符测试 ──
add_executable(
    auto_buff_test
    tests/auto_buff_test.cpp
)

target_link_libraries(
    auto_buff_test
    PRIVATE
        io
        serial
        tools
        auto_aim
        auto_buff
        tinympcstatic
        ${OpenCV_LIBS}
        ${MVS_CAMERA_LIB}
        yaml-cpp
)

if(OpenVINO_FOUND)
    target_link_libraries(
        auto_buff_test
        PRIVATE
            openvino::runtime
    )
endif()

if(Ceres_FOUND)
    target_link_libraries(
        auto_buff_test
        PRIVATE
            ${CERES_LIBRARIES}
    )
endif()

# ── 其他测试 ──
add_executable(
    dm_test
    tests/dm_test.cpp
)

add_executable(
    handeye_test
    tests/handeye_test.cpp
)

add_executable(
    camera_detect_test
    tests/camera_detect_test.cpp
)

add_executable(
    usbcamera_detect_test
    tests/usbcamera_detect_test.cpp
)

add_executable(
    detector_video_test
    tests/detector_video_test.cpp
)

add_executable(
    multi_usbcamera_test
    tests/multi_usbcamera_test.cpp
)

add_executable(
    minimum_vision_system
    tests/minimum_vision_system.cpp
)

foreach(misc_target
    dm_test
    handeye_test
    camera_detect_test
    usbcamera_detect_test
    detector_video_test
    multi_usbcamera_test
    minimum_vision_system
)
    target_link_libraries(
        ${misc_target}
        PRIVATE
            io
            serial
            tools
            auto_aim
            tinympcstatic
            ${OpenCV_LIBS}
            ${MVS_CAMERA_LIB}
            yaml-cpp
    )

    if(OpenVINO_FOUND)
        target_link_libraries(
            ${misc_target}
            PRIVATE
                openvino::runtime
        )
    endif()
endforeach()

# ==========================================
# 标定工具
# ==========================================
add_executable(
    calibrate_camera
    calibration/calibrate_camera.cpp
)

add_executable(
    calibrate_handeye
    calibration/calibrate_handeye.cpp
)

add_executable(
    calibrate_robotworld_handeye
    calibration/calibrate_robotworld_handeye.cpp
)

add_executable(
    capture
    calibration/capture.cpp
)

add_executable(
    split_video
    calibration/split_video.cpp
)

foreach(calib_target
    calibrate_camera
    calibrate_handeye
    calibrate_robotworld_handeye
    capture
    split_video
)
    target_link_libraries(
        ${calib_target}
        PRIVATE
            io
            serial
            tools
            ${OpenCV_LIBS}
            ${MVS_CAMERA_LIB}
            yaml-cpp
    )
endforeach()

# ==========================================
# RPATH
# 让运行时能够找到海康相机 SDK 的动态库
# ==========================================
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(MVS_LIB_DIR "${MVS_ROOT}/lib/64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(MVS_LIB_DIR "${MVS_ROOT}/lib/arm64")
else()
    set(MVS_LIB_DIR "${MVS_ROOT}/lib")
endif()

set(CMAKE_BUILD_RPATH "${MVS_LIB_DIR}")
set(CMAKE_INSTALL_RPATH "${MVS_LIB_DIR}")
