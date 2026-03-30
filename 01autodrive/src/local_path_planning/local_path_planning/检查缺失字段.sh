#!/bin/bash

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 检查 local_path_planning 代码需要哪些配置字段
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  检查 local_path_planning.py 需要的配置字段"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# 查找代码中所有 yaml_data.xxx 的使用
echo "代码中需要的配置字段："
echo "----------------------------------------"

cd /home/nvidia/AutoDrive

# 从安装目录中提取（因为这是实际运行的代码）
PYTHON_FILE="install/local_path_planning/lib/python3.8/site-packages/local_path_planning/local_path_planning.py"

if [ -f "$PYTHON_FILE" ]; then
    # 提取所有 yaml_data.xxx 的字段名
    grep -o "yaml_data\.[a-zA-Z_][a-zA-Z0-9_]*" "$PYTHON_FILE" | \
        sed 's/yaml_data\.//' | \
        sort -u | \
        nl

    echo ""
    echo "总共需要 $(grep -o "yaml_data\.[a-zA-Z_][a-zA-Z0-9_]*" "$PYTHON_FILE" | sed 's/yaml_data\.//' | sort -u | wc -l) 个字段"
else
    echo "错误: 找不到 $PYTHON_FILE"
    echo "请先运行: colcon build --packages-select local_path_planning"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
