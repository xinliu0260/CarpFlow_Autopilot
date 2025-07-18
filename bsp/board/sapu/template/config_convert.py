#!/usr/bin/env python3
# 将 .config 文件转换为 rtconfig.h 头文件
import os
import re

def convert_config_to_rtconfig(config_path, rtconfig_path):
    """
    转换 .config 到 rtconfig.h
    :param config_path: .config 文件路径
    :param rtconfig_path: 生成的 rtconfig.h 路径
    """
    # 读取 .config 内容
    with open(config_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    # 解析配置项（过滤注释和空行）
    config_items = []
    for line in lines:
        line = line.strip()
        # 跳过注释和空行
        if not line or line.startswith('#'):
            # 处理被注释的配置项（如 # CONFIG_XXX is not set）
            if line.startswith('# CONFIG_') and 'is not set' in line:
                key = re.sub(r'^# CONFIG_(.*) is not set$', r'\1', line)
                config_items.append((key, 'n'))  # 'n' 表示未启用
            continue
        # 处理有效配置项（如 CONFIG_XXX=y 或 CONFIG_XXX=32）
        if '=' in line:
            key, value = line.split('=', 1)
            if key.startswith('CONFIG_'):
                key = key[7:]  # 移除前缀 CONFIG_
                config_items.append((key, value.strip()))

    # 生成 config.h 内容
    rtconfig_content = [
        '#ifndef __CONFIG_H__',
        '#define __CONFIG_H__',
        '/* 自动生成的配置文件，请勿手动修改 */',
        ''
    ]

    for key, value in config_items:
        if value == 'y':
            # 布尔型配置（启用）：#define XXX 1
            rtconfig_content.append(f'#define {key} 1')
        elif value == 'n':
            # 布尔型配置（未启用）：不定义或 #undef（根据需求选择）
            # 这里选择不生成，避免宏冲突；若需显式关闭可改为 #undef {key}
            pass
        else:
            # 数值/字符串配置：#define XXX value（如 32 或 "string"）
            rtconfig_content.append(f'#define {key} {value}')

    rtconfig_content.extend([
        '',
        '#endif /* __CONFIG_H__ */'
    ])

    # 写入 rtconfig.h
    with open(rtconfig_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(rtconfig_content))

    print(f"已生成 config.h: {rtconfig_path}")

if __name__ == '__main__':
    # 默认路径
    current_file_path = os.path.abspath(__file__)
    current_dir_path = os.path.dirname(current_file_path)

    config_file = os.path.join(current_dir_path, '.config')
    rtconfig_file = os.path.join(current_dir_path, 'config.h')

    # 若命令行指定路径，则覆盖默认值
    import sys
    if len(sys.argv) == 3:
        config_file = sys.argv[1]
        rtconfig_file = sys.argv[2]

    # 检查 .config 是否存在
    if not os.path.exists(config_file):
        print(f"错误：.config 文件不存在 - {config_file}")
        sys.exit(1)

    # 执行转换
    convert_config_to_rtconfig(config_file, rtconfig_file)