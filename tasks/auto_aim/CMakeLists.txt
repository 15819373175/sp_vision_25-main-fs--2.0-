cmake_minimum_required(VERSION 3.16)

# 查找依赖
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# TinyMPC
add_subdirectory(planner/tinympc)

# 自瞄模块
add_library(auto_aim OBJECT
    armor.cpp
    classifier.cpp
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

# 头文件搜索路径
target_include_directories(auto_aim PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

# 链接依赖
target_link_libraries(auto_aim PUBLIC
    io
    openvino::runtime
    tinympcstatic
    Eigen3::Eigen
    ${OpenCV_LIBS}
    spdlog::spdlog
    fmt::fmt
)
