#!/bin/bash

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}          机器人一键智能编译脚本           ${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""

# --- 智能获取当前工作空间根目录 ---
# 如果当前目录包含 src 文件夹，则使用当前目录；否则尝试默认路径
if [ -d "src" ]; then
    WS_DIR="$(pwd)"
    echo -e "${BLUE}>> 检测到当前目录为工作空间: ${WS_DIR}${NC}"
else
    echo -e "${RED}❌ 错误: 当前目录下没有找到 'src' 文件夹！${NC}"
    echo -e "${YELLOW}请确保你在 workspace 根目录 (如 ~/test_ws) 下运行此脚本。${NC}"
    exit 1
fi

cd "$WS_DIR" || exit 1

# --- 第一步：环境准备 ---
echo -e "${GREEN}[步骤 1/4] 加载 ROS 2 环境...${NC}"
source /opt/ros/humble/setup.bash
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

# --- 第二步：修复 RRT 竞态条件 (预处理) ---
echo -e "${YELLOW}[步骤 2/4] 检测并修复 RRT 消息包依赖 (防止竞态条件)...${NC}"

# 检查 src 下是否有 rrt 相关的包
if [ -d "src/wheeltec_rrt_msg" ]; then
    echo -e "${BLUE}>> 发现 wheeltec_rrt_msg，正在单独预编译...${NC}"
    
    # 单独编译消息包
    colcon build --packages-select wheeltec_rrt_msg --symlink-install --event-handlers=console_direct+
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}❌ 失败: 消息包编译出错，停止后续操作。${NC}"
        exit 1
    fi
    
    # 刷新环境
    source "$WS_DIR/install/setup.bash"
    echo -e "${GREEN}✅ 消息包预编译成功，依赖已就绪。${NC}"
else
    echo -e "${BLUE}>> 未发现 wheeltec_rrt_msg，跳过预编译步骤。${NC}"
fi

echo ""

# --- 第三步：执行用户想要的标准编译 ---
echo -e "${GREEN}[步骤 3/4] 开始执行标准全量编译 (colcon build)...${NC}"
echo -e "${YELLOW}>> 提示: 这将编译所有未完成的包，请耐心等待...${NC}"
echo ""

# 执行你习惯的命令 (就在当前目录运行)
colcon build 

# 检查编译结果
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}=========================================${NC}"
    echo -e "${GREEN}   🎉 恭喜！全量编译成功完成！         ${NC}"
    echo -e "${GREEN}=========================================${NC}"
    echo ""
    echo -e "${BLUE}下一步操作建议:${NC}"
    echo -e "1. 加载环境: ${GREEN}source install/setup.bash${NC}"
    echo -e "2. 启动机器人: ${GREEN}ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py${NC}"
else
    echo ""
    echo -e "${RED}=========================================${NC}"
    echo -e "${RED}   ⚠️ 编译过程中出现错误，请检查上方日志  ${NC}"
    echo -e "${RED}=========================================${NC}"
    exit 1
fi