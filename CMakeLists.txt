cmake_minimum_required(VERSION 3.8)
project(USB2CAN_example)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找线程库
find_package(Threads REQUIRED)

# 添加可执行文件
add_executable(usb2can_example
  src/main.cpp
  src/can_usb_device.cpp
)

# 设置头文件目录
target_include_directories(usb2can_example PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 链接线程库
target_link_libraries(usb2can_example
  Threads::Threads
)

# 可选：安装目标（如果不需要可以删除）
install(TARGETS usb2can_example
  DESTINATION bin
)

# 可选：安装头文件（如果不需要可以删除）
install(DIRECTORY include/
  DESTINATION include
)