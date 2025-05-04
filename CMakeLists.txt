cmake_minimum_required(VERSION 3.10)
project(usb2can)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

add_library(usb2can STATIC
    src/usb2can.cpp
)

add_executable(main src/main.cpp)

target_link_libraries(main usb2can pthread)
