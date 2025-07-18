message("add STM32CubeMX generate files to ${PROJECT_NAME}")
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/Core/Inc/)
target_sources(${PROJECT_NAME} PRIVATE 
        ${CMAKE_CURRENT_LIST_DIR}/Core/Src/system_stm32h7xx.c
        ${CMAKE_CURRENT_LIST_DIR}/Core/Src/stm32h7xx_hal_msp.c
)