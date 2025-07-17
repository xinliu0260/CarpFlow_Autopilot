# 1. 基础配置：获取当前目录路径
set(HAL_ROOT ${CMAKE_CURRENT_LIST_DIR})  # 当前CMake文件所在目录（对应SCons的cwd）

# 2. 从rtconfig.h中读取配置项（需确保rtconfig.h已生成且可被CMake识别）
# 注：需在主CMakeLists.txt中先包含rtconfig.h所在目录（如${CMAKE_BINARY_DIR}）
include(CheckSymbolExists)

# 辅助函数：检查rtconfig.h中是否定义了某个宏
function(check_rt_config SYMBOL RESULT_VAR)
    check_symbol_exists(${SYMBOL} "rtconfig.h" ${RESULT_VAR})
endfunction()

# 3. 检查核心配置项，决定HAL库文件集合
check_rt_config(ADD_ALL_MCU_SDK ADD_ALL_MCU_SDK)

# 初始化源文件列表
set(HAL_SRC "")

# 3.1 如果ADD_ALL_MCU_SDK为真：添加所有HAL源文件（剔除模板）
if(ADD_ALL_MCU_SDK)
    # 获取所有HAL驱动源文件
    file(GLOB HAL_DRIVER_SRC "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/*.c")
    # 剔除模板文件（匹配*template.c）
    list(FILTER HAL_DRIVER_SRC EXCLUDE REGEX ".*template\\.c$")
    # 添加系统文件
    list(APPEND HAL_SRC
        ${HAL_DRIVER_SRC}
        "${HAL_ROOT}/CMSIS/Device/ST/STM32H7xx/Source/Templates/system_stm32h7xx.c"
    )

# 3.2 否则：按需添加基础文件+条件文件
else()
    # 基础源文件（对应SCons中的Split列表）
    set(BASIC_SRC
        "${HAL_ROOT}/CMSIS/Device/ST/STM32H7xx/Source/Templates/system_stm32h7xx.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cec.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_comp.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_crc.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_crc_ex.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cryp.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cryp_ex.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rng.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sram.c"
        "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c"
    )
    list(APPEND HAL_SRC ${BASIC_SRC})

    # 条件添加：根据rtconfig.h中的配置项添加对应源文件
    # 串口相关
    check_rt_config(RT_USING_SERIAL HAS_SERIAL)
    check_rt_config(RT_USING_NANO HAS_NANO)
    check_rt_config(RT_USING_CONSOLE HAS_CONSOLE)
    if(HAS_SERIAL OR (HAS_NANO AND HAS_CONSOLE))
        list(APPEND HAL_SRC
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c"
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_usart.c"
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c"
        )
    endif()

    # I2C相关
    check_rt_config(RT_USING_I2C HAS_I2C)
    if(HAS_I2C)
        list(APPEND HAL_SRC
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c"
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c"
        )
    endif()

    # SPI相关
    check_rt_config(RT_USING_SPI HAS_SPI)
    if(HAS_SPI)
        list(APPEND HAL_SRC
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c"
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_qspi.c"
        )
    endif()

    # USB相关
    check_rt_config(RT_USING_USB HAS_USB)
    if(HAS_USB)
        list(APPEND HAL_SRC
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c"
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c"
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hcd.c"
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c"
        )
    endif()

    # CAN相关
    check_rt_config(RT_USING_CAN HAS_CAN)
    if(HAS_CAN)
        list(APPEND HAL_SRC
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c"
        )
    endif()

    # 定时器/PWM相关
    check_rt_config(RT_USING_HWTIMER HAS_HWTIMER)
    check_rt_config(RT_USING_PWM HAS_PWM)
    if(HAS_HWTIMER OR HAS_PWM)
        list(APPEND HAL_SRC
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c"
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c"
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_lptim.c"
        )
    endif()

    # 以太网相关
    check_rt_config(BSP_USING_ETH HAS_ETH)
    check_rt_config(BSP_USING_ETH_H750 HAS_ETH_H750)
    if(HAS_ETH OR HAS_ETH_H750)
        list(APPEND HAL_SRC
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_eth.c"
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_eth_ex.c"
        )
    endif()

    # 其他外设（ADC/DAC/RTC等）按同样逻辑添加
    check_rt_config(RT_USING_ADC HAS_ADC)
    if(HAS_ADC)
        list(APPEND HAL_SRC
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c"
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c"
        )
    endif()

    check_rt_config(RT_USING_DAC HAS_DAC)
    if(HAS_DAC)
        list(APPEND HAL_SRC
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dac.c"
            "${HAL_ROOT}/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dac_ex.c"
        )
    endif()

    # 更多外设配置项（省略部分与原SCons一致的逻辑，按需补充）
endif()

# 4. 设置HAL库头文件路径
set(HAL_INC_PATH
    "${HAL_ROOT}/STM32H7xx_HAL_Driver/Inc"
    "${HAL_ROOT}/CMSIS/Device/ST/STM32H7xx/Include"
)

# 5. 设置HAL库宏定义
set(HAL_DEFINES
    "USE_HAL_DRIVER"
    "USE_FULL_LL_DRIVER"
)

# 6. 创建HAL库目标（供主工程链接）
add_library(stm32_hal STATIC ${HAL_SRC})
target_include_directories(stm32_hal PUBLIC ${HAL_INC_PATH})
target_compile_definitions(stm32_hal PUBLIC ${HAL_DEFINES})

# 7. 处理ARM DSP库（条件包含）
check_rt_config(USE_ARM_DSP HAS_ARM_DSP)
if(HAS_ARM_DSP)
    # DSP库头文件路径
    set(DSP_INC "${HAL_ROOT}/CMSIS/DSP/Include")
    # 根据编译器设置库路径和库名
    if(CMAKE_C_COMPILER_ID MATCHES "ARMCC|ARMClang")
        set(DSP_LIB_PATH "${HAL_ROOT}/CMSIS/DSP/Lib/ARM")
        set(DSP_LIB "arm_cortexM7lfdp_math")
    elseif(CMAKE_C_COMPILER_ID MATCHES "GNU")
        set(DSP_LIB_PATH "${HAL_ROOT}/CMSIS/DSP/Lib/GCC")
        set(DSP_LIB "arm_cortexM7lfdp_math")
    endif()

    # 添加DSP库到HAL目标
    target_include_directories(stm32_hal PUBLIC ${DSP_INC})
    target_link_directories(stm32_hal PUBLIC ${DSP_LIB_PATH})
    target_link_libraries(stm32_hal PUBLIC ${DSP_LIB})
    target_compile_definitions(stm32_hal PUBLIC "ARM_MATH_CM7")
endif()

# 8. 向主工程暴露HAL目标（类似SCons的Return('group')）
set(STM32_HAL_TARGET stm32_hal PARENT_SCOPE)