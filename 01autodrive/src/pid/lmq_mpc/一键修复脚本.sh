#!/bin/bash
# 一键修复脚本 - 解决 colcon build 的所有依赖问题
# 使用方法: bash 一键修复脚本.sh

set -e  # 遇到错误立即退出

echo "========================================"
echo "   一键修复 ROS 2 编译环境"
echo "========================================"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 1. 降级 setuptools 到稳定版本（最重要）
echo -e "\n${BLUE}[1/5] 降级 setuptools 到稳定版本...${NC}"
if sudo pip3 install setuptools==58.2.0 2>/dev/null; then
    echo -e "${GREEN}✓ setuptools 降级成功${NC}"
else
    echo -e "${YELLOW}⚠ sudo 失败，尝试用户安装...${NC}"
    pip3 install --user setuptools==58.2.0
fi

# 验证 setuptools
if python3 -c "from setuptools import setup; print('OK')" 2>/dev/null; then
    echo -e "${GREEN}✓ setuptools 验证通过${NC}"
else
    echo -e "${RED}✗ setuptools 仍有问题，尝试同时降级 packaging...${NC}"
    sudo pip3 install setuptools==58.2.0 packaging==21.3 || \
        pip3 install --user setuptools==58.2.0 packaging==21.3
fi

# 2. 创建 __init__.py 文件
echo -e "\n${BLUE}[2/5] 创建必要的 __init__.py 文件...${NC}"
cd /home/nvidia/AutoDrive/src/pid

# 创建 pid.new 包（兼容性）
if [ ! -f "pid/new/__init__.py" ]; then
    touch pid/new/__init__.py
    echo -e "${GREEN}✓ 创建 pid/new/__init__.py${NC}"
fi

# 询问是否创建 lmq_mpc 包
echo -e "\n${YELLOW}是否创建新的 lmq_mpc 包（推荐）？${NC}"
echo "  [y] 是 - 创建独立的 lmq_mpc 包（结构更清晰）"
echo "  [n] 否 - 仅使用 pid.new（最小改动）"
read -p "请选择 [y/n]: " create_lmq

if [[ "$create_lmq" =~ ^[Yy]$ ]]; then
    mkdir -p lmq_mpc
    touch lmq_mpc/__init__.py

    if [ -f "pid/new/lmq_mpc_plf.py" ]; then
        cp pid/new/lmq_mpc_plf.py lmq_mpc/
        cp pid/new/sim_platoon_test_plf.py lmq_mpc/ 2>/dev/null || true
        echo -e "${GREEN}✓ 创建 lmq_mpc 包并复制文件${NC}"
    fi
fi

# 3. 检查 setup.py
echo -e "\n${BLUE}[3/5] 检查 setup.py 配置...${NC}"
if grep -q "'lmq_mpc'" setup.py 2>/dev/null; then
    echo -e "${GREEN}✓ setup.py 包含 lmq_mpc 包声明${NC}"
elif grep -q "f'{package_name}.new'" setup.py 2>/dev/null; then
    echo -e "${GREEN}✓ setup.py 包含 pid.new 包声明${NC}"
else
    echo -e "${RED}✗ setup.py 可能需要手动修改${NC}"
    echo "  请确保 packages 列表包含: f'{package_name}.new'"
fi

# 4. 清理并编译
echo -e "\n${BLUE}[4/5] 清理旧文件并重新编译...${NC}"
cd /home/nvidia/AutoDrive
rm -rf build/pid install/pid log/ 2>/dev/null || true

echo -e "${YELLOW}开始编译...${NC}"
if colcon build --packages-select pid 2>&1 | tee /tmp/colcon_build.log; then
    echo -e "${GREEN}✓ 编译成功！${NC}"

    # 加载环境
    source install/setup.bash

    # 5. 验证
    echo -e "\n${BLUE}[5/5] 验证安装...${NC}"
    echo -e "${GREEN}可用的可执行文件:${NC}"
    ros2 pkg executables pid

    echo -e "\n${GREEN}========================================"
    echo "✓ 修复完成！"
    echo "========================================"
    echo ""
    echo "现在可以使用以下方式运行:"
    echo ""
    echo "1. ROS 2 节点方式:"
    if [[ "$create_lmq" =~ ^[Yy]$ ]]; then
        echo -e "   ${BLUE}ros2 run pid lmq_mpc${NC}"
    fi
    echo -e "   ${BLUE}ros2 run pid lmq_mpc1${NC}"
    echo ""
    echo "2. Python 直接运行（无需编译，推荐测试时使用）:"
    if [[ "$create_lmq" =~ ^[Yy]$ ]]; then
        echo -e "   ${BLUE}python3 -u src/pid/lmq_mpc/lmq_mpc_plf.py${NC}"
    fi
    echo -e "   ${BLUE}python3 -u src/pid/pid/new/lmq_mpc_plf.py${NC}"
    echo ""
    echo "3. 查看操作指南:"
    echo "   src/pid/pid/new/三车PLF编队实验操作指南.md"
    echo "========================================${NC}"

else
    echo -e "${RED}✗ 编译失败${NC}"
    echo ""
    echo -e "${YELLOW}请检查错误信息:${NC}"
    tail -n 20 /tmp/colcon_build.log
    echo ""
    echo -e "${YELLOW}临时解决方案 - 直接运行（跳过编译）:${NC}"
    if [[ "$create_lmq" =~ ^[Yy]$ ]]; then
        echo "  cd /home/nvidia/AutoDrive"
        echo "  python3 -u src/pid/lmq_mpc/lmq_mpc_plf.py"
    else
        echo "  cd /home/nvidia/AutoDrive"
        echo "  python3 -u src/pid/pid/new/lmq_mpc_plf.py"
    fi
    echo ""
    echo "查看详细文档: 修复colcon编译错误.md"
    exit 1
fi
