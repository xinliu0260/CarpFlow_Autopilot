cmake_minimum_required(VERSION 3.30)
# 禁止 CMake 检测编译器是否能生成主机可执行程序
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/arm-none-eabi-toolchain.cmake")

project(CarpFlow_Autopilot LANGUAGES C CXX ASM)

add_subdirectory("bsp/board/sapu/template")


