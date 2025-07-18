
set(PROJECT_NAME sapu)
set(PROJECT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
message("项目路径：${PROJECT_PATH}")
set(CORSS_TOOL "gcc")
set(CPU cortex-m7)
include(CheckSymbolExists)
# 辅助函数：检查rtconfig.h中是否定义了某个宏
function(check_rt_config SYMBOL RESULT_VAR)
    check_symbol_exists(${SYMBOL} "${PROJECT_PATH}/rtconfig.h" ${RESULT_VAR})
endfunction()

# 创建目标 SAPU
add_executable(${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")

include(cmake/cortex_m7.cmake)
include(cmake/config_convert.cmake)
include(${PROJECT_PATH}/driver/driver.cmake)
include(startup/startup.cmake)
include(${PROJECT_PATH}/CubeMX_Config/stm32cubemx.cmake)
include(${CMAKE_SOURCE_DIR}/bsp/libraries/stm32/STM32H7xx_HAL/stm32h7xx_hal.cmake)
include(${CMAKE_SOURCE_DIR}/bsp/libraries/stm32/STM32H7xx_HAL/cmsis.cmake)
include(${CMAKE_SOURCE_DIR}/rt-thread/rt-thread.cmake)

target_compile_definitions(${PROJECT_NAME} PRIVATE STM32H743xx)
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_PATH})

# message("${CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES}")
# get_target_property(inc ${PROJECT_NAME} INCLUDE_DIRECTORIES)
# message("${inc}")



