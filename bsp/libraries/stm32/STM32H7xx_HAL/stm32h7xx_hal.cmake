# 1. 基础配置：获取当前目录路径
set(HAL_ROOT ${CMAKE_CURRENT_LIST_DIR})  # 当前CMake文件所在目录（对应SCons的cwd）

# 3. 检查核心配置项，决定HAL库文件集合
check_rt_config(ADD_ALL_HAL_LIB HAS_ADD_ALL_HAL_LIB)
# 初始化源文件列表
set(HAL_SRC "")
# 3.1 如果ADD_ALL_HAL_LIB为真：添加所有HAL源文件（剔除模板）
if(HAS_ADD_ALL_HAL_LIB)
    # 获取所有HAL驱动源文件
    message("add STM32H7xx HAL Driver to ${PROJECT_NAME}")
    file(GLOB HAL_DRIVER_SRC "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/*.c")
    # 剔除模板文件（匹配*template.c）
    list(FILTER HAL_DRIVER_SRC EXCLUDE REGEX ".*template\\.c$")
    # 添加系统文件
    list(APPEND HAL_SRC
        ${HAL_DRIVER_SRC}
    )
else()
# 3.2 否则根据conf.h中进行条件添加

endif()
target_sources(${PROJECT_NAME} PRIVATE ${HAL_SRC})

# 4. 设置HAL库头文件路径
set(HAL_INC_PATH
    "${HAL_ROOT}/STM32H7xx_HAL_Driver/Inc" )
target_include_directories(${PROJECT_NAME} PRIVATE ${HAL_INC_PATH})

# 5. 设置HAL库宏定义
set(HAL_DEFINES
    "USE_HAL_DRIVER"
    "USE_FULL_LL_DRIVER"
)
target_compile_definitions(${PROJECT_NAME} PRIVATE ${HAL_DEFINES})