cmake_minimum_required(VERSION 3.16)
project(minimal_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# ---- OpenCV ----
find_package(OpenCV REQUIRED)

# ---- OpenVINO ----
# Adjust this path if your OpenVINO is installed elsewhere
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

add_executable(minimal_test minimal_test.cpp)

target_include_directories(minimal_test PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(minimal_test
    PRIVATE
    ${OpenCV_LIBS}
    openvino::runtime
)
