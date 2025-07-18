message("add startup source to ${PROJECT_NAME}")
set(STARTUP_ASM "")

if(CORSS_TOOL STREQUAL "gcc")
    set(STARTUP_ASM ${CMAKE_CURRENT_LIST_DIR}/gcc/startup_stm32h743xx.s)
    set(LINK_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/gcc/link.lds)
    target_link_options(${PROJECT_NAME} PRIVATE -T ${LINK_SCRIPT})
elseif(CORSS_TOOL STREQUAL "armclang")
    set(STARTUP_ASM ${CMAKE_CURRENT_LIST_DIR}/arm/startup_stm32h743xx.s)
    set(LINK_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/arm/link.sct)
endif()

target_sources(${PROJECT_NAME} PRIVATE ${STARTUP_ASM})

