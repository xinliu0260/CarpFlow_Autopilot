message("add cmsis-dsp to ${PROJECT_NAME}")
# 处理ARM DSP库
check_rt_config(USE_ARM_DSP HAS_ARM_DSP)
if(HAS_ARM_DSP)
    # DSP库头文件路径
    set(DSP_INC "${CMAKE_CURRENT_LIST_DIR}/CMSIS/DSP/Include" )
    # 根据编译器设置库路径和库名
    if(CORSS_TOOL MATCHES "armcc|armclang")
        set(DSP_LIB_PATH "${CMAKE_CURRENT_LIST_DIR}/CMSIS/DSP/Lib/ARM" )
        set(DSP_LIB "arm_cortexM7lfdp_math" )
    elseif(CORSS_TOOL MATCHES "gcc")
        set(DSP_LIB_PATH "${CMAKE_CURRENT_LIST_DIR}/CMSIS/DSP/Lib/GCC" )
        set(DSP_LIB "arm_cortexM7lfdp_math" ) 
    endif()
    target_link_libraries(${PROJECT_NAME} PRIVATE ${DSP_LIB})
    target_link_directories(${PROJECT_NAME} PRIVATE ${DSP_LIB_PATH})
    target_include_directories(${PROJECT_NAME} PRIVATE ${DSP_INC})
endif()
message("add cmsis-core to ${PROJECT_NAME}")
target_include_directories(${PROJECT_NAME} PRIVATE 
                ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32H7xx/Include
                ${CMAKE_CURRENT_LIST_DIR}/CMSIS/core/include
                )
