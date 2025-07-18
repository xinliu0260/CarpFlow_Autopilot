set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

find_program(COMPILER_ON_PATH "arm-none-eabi-gcc.exe")

if(COMPILER_ON_PATH)
    get_filename_component(ARM_TOOLCHAIN_PATH ${COMPILER_ON_PATH} DIRECTORY)
    message(STATUS "Using ARM GCC from path = ${ARM_TOOLCHAIN_PATH}")
else()
    message(FATAL_ERROR "Unable to find ARM GCC (arm-none-eabi-gcc.exe). Add to your PATH")
endif()

set(CMAKE_C_COMPILER ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-gcc.exe)
set(CMAKE_CXX_COMPILER ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-g++.exe)
set(CMAKE_ASM_COMPILER ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-gcc.exe)
set(CMAKE_LINKER ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-gcc.exe)
set(CMAKE_CPP ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-cpp.exe)
set(CMAKE_SIZE_UTIL ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-size.exe)
set(CMAKE_OBJCOPY ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-objcopy.exe)
set(CMAKE_OBJDUMP ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-objdump.exe)
set(CMAKE_NM_UTIL ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-gcc-nm.exe)
set(CMAKE_AR ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-gcc-ar.exe)
set(CMAKE_RANLIB ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-gcc-ranlib.exe)

# # 禁用 CMake 对 Windows 系统库的自动链接
# set(CMAKE_C_IMPLICIT_LINK_LIBRARIES "")
# set(CMAKE_CXX_IMPLICIT_LINK_LIBRARIES "")
# set(CMAKE_EXE_LINKER_FLAGS_INIT "-nostdlib -nostartfiles")

# # 禁用共享库（嵌入式项目不需要 DLL）
# set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# 告诉 CMake 不要在主机系统（Windows）上查找库和头文件
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)