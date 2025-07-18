message("add startup source to ${PROJECT_NAME}")

if(CORSS_TOOL STREQUAL "gcc")
    target_sources(${PROJECT_NAME} PRIVATE 
                "${CMAKE_CURRENT_LIST_DIR}/gcc/startup_stm32h743xx.s"
                "${CMAKE_CURRENT_LIST_DIR}/gcc/startup.c"
                )
    target_link_options(${PROJECT_NAME} PRIVATE -T ${CMAKE_CURRENT_LIST_DIR}/gcc/link.lds)

elseif(CORSS_TOOL STREQUAL "armclang")
    target_sources(${PROJECT_NAME} PRIVATE 
                "${CMAKE_CURRENT_LIST_DIR}/arm/startup_stm32h743xx.s"
                "${CMAKE_CURRENT_LIST_DIR}/arm/startup.c"
                )
    # todo armclang分散加载脚本链接配置
    set(LINK_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/arm/link.sct)
    
endif()



