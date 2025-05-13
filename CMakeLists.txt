cmake_minimum_required(VERSION 3.10)
project(TestCanUsbDevice)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 头文件路径
include_directories(${PROJECT_SOURCE_DIR}/include)

# 链接目录
link_directories(${PROJECT_SOURCE_DIR}/lib)

# 可执行文件
add_executable(test_canusb src/main.cpp)

# 链接动态库
target_link_libraries(test_canusb PRIVATE CanUsbDevice pthread)