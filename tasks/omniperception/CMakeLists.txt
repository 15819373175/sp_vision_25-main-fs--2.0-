cmake_minimum_required(VERSION 3.16)

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(omniperception OBJECT
    decider.cpp
    perceptron.cpp
)

target_include_directories(omniperception PRIVATE
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(omniperception PRIVATE
    openvino::runtime
)
