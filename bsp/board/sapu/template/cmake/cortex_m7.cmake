# 根据内核设置工具链选项
set(CPU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")

set(SPECS_FLAGS "--specs=nosys.specs --specs=nano.specs")
set(COMMON_FLAGS "-ffunction-sections -fdata-sections -Wall -Wdouble-promotion -Wno-sign-compare -Wno-psabi -g3 -ggdb3")
set(ASM_FLAGS "-x assembler-with-cpp")
set(CXX_FLAGS "-fno-rtti -fno-exceptions -fno-threadsafe-statics -Wsuggest-override -Wno-register")

set(CMAKE_C_FLAGS "${CPU_FLAGS} ${COMMON_FLAGS} ${SPECS_FLAGS}")
set(CMAKE_CXX_FLAGS "${CPU_FLAGS} ${COMMON_FLAGS} ${SPECS_FLAGS} ${CXX_FLAGS}")
set(CMAKE_ASM_FLAGS "${CPU_FLAGS} ${SPECS_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections,--print-memory-usage,-Map=${PROJECT_NAME}.map,-cref")

