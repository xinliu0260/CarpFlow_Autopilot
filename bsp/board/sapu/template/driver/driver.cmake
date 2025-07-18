
set(DRIVER_INC ${CMAKE_CURRENT_LIST_DIR}/inc/)
file(GLOB DRIVER_SRC ${CMAKE_CURRENT_LIST_DIR}/src/*.c)
target_sources(${PROJECT_NAME} PRIVATE ${DRIVER_SRC})
target_include_directories(${PROJECT_NAME} PRIVATE ${DRIVER_INC})
target_link_options(${PROJECT_NAME} PRIVATE -T ${LINK_SCRIPT})
