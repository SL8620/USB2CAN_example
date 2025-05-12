cmake_minimum_required(VERSION 3.10)
project(CanUsb)

set(CMAKE_CXX_STANDARD 17)

add_executable(CanUsb
    main.cpp
    CanUsbDevice.cpp
)
