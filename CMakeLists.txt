# 最低CMake版本要求
cmake_minimum_required(VERSION 3.8)

# 项目名称
project(USB2CAN_example LANGUAGES CXX)

# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # 禁用编译器扩展，确保标准兼容性

# 检查操作系统（确保Linux环境，preempt-rt依赖）
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "This project requires Linux (preempt-rt recommended)")
endif()

# 查找线程库
find_package(Threads REQUIRED)

# 设置编译选项
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-g -Wall -Wextra -Wpedantic) # 调试模式：启用警告和调试符号
else()
  add_compile_options(-O3 -DNDEBUG -Wall -Wextra -Wpedantic) # 发布模式：高优化，无调试符号
endif()
add_compile_options(-pthread) # 显式启用pthread支持

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
target_link_libraries(usb2can_example PRIVATE
  Threads::Threads
)

# 可选：安装目标（注释掉，若无需安装到系统）
# install(TARGETS usb2can_example
#   DESTINATION bin
# )
# install(DIRECTORY include/
#   DESTINATION include
# )

# 打印构建信息
message(STATUS "Building USB2CAN_example with CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")