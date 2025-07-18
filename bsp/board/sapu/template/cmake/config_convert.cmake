set(MENUCONFIG_PATH ${PROJECT_PATH}/.config)
set(CONFIG_H_PATH ${PROJECT_PATH}/rtconfig.h)

add_custom_command(
    OUTPUT ${CONFIG_H_PATH}
    COMMAND python 
    "${PROJECT_PATH}/config_convert.py"  # 脚本路径
    "${MENUCONFIG_PATH}" 
    "${CONFIG_H_PATH}"
  DEPENDS ${MENUCONFIG_PATH}  # .config 变化时重新生成
  COMMENT "convert .config file to rtconfig.h"
)


add_custom_target(
  generate_config ALL  # 目标名称为generate_rtconfig，ALL表示默认构建
  DEPENDS ${CONFIG_H_PATH}  # 依赖rtconfig.h（由上面的custom_command生成）
)

