#!/bin/bash

# RC2026 一键启动脚本
# 功能：启动激光雷达驱动、定位算法(PointLIO/GLIM)、底盘控制与通信、梯田相机与YOLO识别

set -u

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 工作空间路径
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_FILE="$WORKSPACE_DIR/install/setup.bash"
SESSION_NAME="rc2026"

print_header() {
    echo -e "${GREEN}======================================${NC}"
    echo -e "${GREEN}    RC2026 一键启动脚本${NC}"
    echo -e "${GREEN}======================================${NC}"
}

check_command() {
    local cmd="$1"
    local hint="$2"
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo -e "${RED}错误: 缺少命令 $cmd${NC}"
        echo -e "${YELLOW}$hint${NC}"
        exit 1
    fi
}

launch_exists() {
    local pkg="$1"
    local launch_file="$2"
    local pkg_prefix

    pkg_prefix="$(ros2 pkg prefix "$pkg" 2>/dev/null || true)"
    if [ -z "$pkg_prefix" ]; then
        return 1
    fi

    [ -f "$pkg_prefix/share/$pkg/launch/$launch_file" ]
}

resolve_exec() {
    local pkg="$1"
    shift

    local available=""
    available="$(ros2 pkg executables "$pkg" 2>/dev/null | awk '{print $2}')"
    if [ -z "$available" ]; then
        return 1
    fi

    local candidate
    for candidate in "$@"; do
        if echo "$available" | grep -qx "$candidate"; then
            echo "$candidate"
            return 0
        fi
    done

    echo "$available" | head -n 1
    return 0
}

new_window_and_run() {
    local index="$1"
    local name="$2"
    local banner="$3"
    local cmd="$4"

    if [ "$index" -eq 0 ]; then
        tmux new-session -d -s "$SESSION_NAME" -n "$name"
    else
        tmux new-window -t "$SESSION_NAME:$index" -n "$name"
    fi

    tmux send-keys -t "$SESSION_NAME:$index" "cd $WORKSPACE_DIR" C-m
    tmux send-keys -t "$SESSION_NAME:$index" "source install/setup.bash" C-m
    tmux send-keys -t "$SESSION_NAME:$index" "echo -e '${GREEN}[启动] $banner${NC}'" C-m
    tmux send-keys -t "$SESSION_NAME:$index" "$cmd" C-m
}

print_header

check_command tmux "请先安装: sudo apt install tmux"

if [ -z "${ROS_DISTRO:-}" ]; then
    echo -e "${RED}错误: 未检测到ROS2环境，请先 source ROS2 (如 /opt/ros/<distro>/setup.bash)${NC}"
    exit 1
fi

if [ ! -f "$SETUP_FILE" ]; then
    echo -e "${RED}错误: 未找到工作空间环境文件: $SETUP_FILE${NC}"
    echo -e "${YELLOW}请先在工作空间根目录执行 colcon build${NC}"
    exit 1
fi

echo -e "${YELLOW}[1/5] Source 工作空间...${NC}"
cd "$WORKSPACE_DIR"
source "$SETUP_FILE"

echo -e "${YELLOW}[2/5] 请选择定位算法:${NC}"
echo "  1) PointLIO (默认)"
echo "  2) GLIM"
read -r -p "请输入选项 [1-2]: " algo_choice

echo ""
echo "请选择启动模式:"
echo "  1) 分模块启动 (原有模式)"
echo "  2) 行为树总控启动 (结合 r2_decision/bt_executor.launch.py，推荐)"
read -r -p "请输入选项 [1-2，默认2]: " startup_choice

ALGO_TYPE="PointLIO"
LIDAR_LAUNCH="msg_MID360_side.launch.py"
LOCALIZATION_CMD="ros2 launch point_lio point_lio.launch.py use_sim_time:=false"
CONTROL_LAUNCH="stm32_control.launch.py"

if [ "$algo_choice" = "2" ]; then
    ALGO_TYPE="GLIM"
    LIDAR_LAUNCH="msg_MID360_side_GLIM.launch.py"
    CONTROL_LAUNCH="stm32_control_GLIM.launch.py"

    GLIM_PREFIX="$(ros2 pkg prefix glim 2>/dev/null || true)"
    if [ -z "$GLIM_PREFIX" ]; then
        echo -e "${RED}错误: 未找到 ROS 包 glim，请确认 GLIM 已成功编译安装${NC}"
        exit 1
    fi

    GLIM_CONFIG_PATH="$GLIM_PREFIX/share/glim/config"
    LOCALIZATION_CMD="ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$GLIM_CONFIG_PATH"
fi

echo -e "${GREEN}已选择: $ALGO_TYPE${NC}"

echo -e "${YELLOW}[3/5] 校验关键启动文件...${NC}"
if ! launch_exists "livox_ros_driver2" "$LIDAR_LAUNCH"; then
    echo -e "${RED}错误: 未找到 livox 启动文件: $LIDAR_LAUNCH${NC}"
    exit 1
fi

if ! launch_exists "stm32_control" "$CONTROL_LAUNCH"; then
    echo -e "${RED}错误: 未找到底盘控制启动文件: $CONTROL_LAUNCH${NC}"
    exit 1
fi

if ! launch_exists "stm32_comm" "stm32_comm.launch.py"; then
    echo -e "${RED}错误: 未找到底盘通信启动文件: stm32_comm.launch.py${NC}"
    exit 1
fi

if ! launch_exists "r2_decision" "bt_executor.launch.py"; then
    echo -e "${RED}错误: 未找到行为树启动文件: r2_decision/bt_executor.launch.py${NC}"
    exit 1
fi

TERRACED_EXEC="$(resolve_exec terraced_camera terraced_camera_node || true)"
if [ -z "$TERRACED_EXEC" ]; then
    echo -e "${RED}错误: 未找到 terraced_camera 可执行节点${NC}"
    exit 1
fi

YOLO_EXEC="$(resolve_exec yolo_spearhead_see yolo_spearhead_see yolo_spearhead_see_node || true)"
if [ -z "$YOLO_EXEC" ]; then
    echo -e "${RED}错误: 未找到 yolo_spearhead_see 可执行节点${NC}"
    exit 1
fi

echo -e "${YELLOW}[4/5] 创建 tmux 会话: $SESSION_NAME${NC}"
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo -e "${YELLOW}检测到已存在会话，正在关闭旧会话...${NC}"
    tmux kill-session -t "$SESSION_NAME"
fi

echo -e "${YELLOW}[5/5] 启动各模块...${NC}"
if [ "$startup_choice" = "1" ]; then
    new_window_and_run 0 "lidar" "激光雷达驱动 ($ALGO_TYPE 模式)" "ros2 launch livox_ros_driver2 $LIDAR_LAUNCH"
    new_window_and_run 1 "localization" "$ALGO_TYPE 定位算法" "$LOCALIZATION_CMD"
    new_window_and_run 2 "control" "底盘控制 ($ALGO_TYPE 模式)" "ros2 launch stm32_control $CONTROL_LAUNCH"
    new_window_and_run 3 "comm" "底盘通信 (公共模块)" "ros2 launch stm32_comm stm32_comm.launch.py"
    new_window_and_run 4 "terraced_camera" "梯田相机驱动" "ros2 run terraced_camera $TERRACED_EXEC --ros-args -p flip_mode:=1"
    new_window_and_run 5 "yolo_spearhead" "YOLO矛头抓取对准识别" "ros2 run yolo_spearhead_see $YOLO_EXEC"
    new_window_and_run 6 "terminal_1" "调试终端" "bash"
    new_window_and_run 7 "terminal_2" "调试终端" "bash"
    WINDOW_HINT="分模块模式"
else
    if [ "$ALGO_TYPE" = "GLIM" ]; then
        BT_LOCALIZATION="glim"
    else
        BT_LOCALIZATION="pointlio"
    fi

    new_window_and_run 0 "lidar" "激光雷达驱动 ($ALGO_TYPE 模式)" "ros2 launch livox_ros_driver2 $LIDAR_LAUNCH"
    new_window_and_run 1 "control" "底盘控制 ($ALGO_TYPE 模式)" "ros2 launch stm32_control $CONTROL_LAUNCH"
    new_window_and_run 2 "decision_bt" "行为树总控" "ros2 launch r2_decision bt_executor.launch.py localization:=$BT_LOCALIZATION"
    new_window_and_run 3 "terminal_1" "调试终端" "bash"
    new_window_and_run 4 "terminal_2" "调试终端" "bash"
    WINDOW_HINT="行为树总控模式"
fi

tmux select-window -t "$SESSION_NAME:0"

echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}启动完成！${NC}"
echo -e "${GREEN}======================================${NC}"
echo -e "启动模式: ${YELLOW}$WINDOW_HINT${NC}"
echo -e "tmux 窗口布局:"
if [ "$startup_choice" = "1" ]; then
    echo -e "  ${YELLOW}窗口0 (lidar)${NC}            - 激光雷达驱动"
    echo -e "  ${YELLOW}窗口1 (localization)${NC}     - $ALGO_TYPE 定位"
    echo -e "  ${YELLOW}窗口2 (control)${NC}          - 底盘控制"
    echo -e "  ${YELLOW}窗口3 (comm)${NC}             - 底盘通信"
    echo -e "  ${YELLOW}窗口4 (terraced_camera)${NC}   - 梯田相机驱动"
    echo -e "  ${YELLOW}窗口5 (yolo_spearhead)${NC}   - YOLO矛头识别"
    echo -e "  ${YELLOW}窗口6/7 (terminal_1/2)${NC}   - 调试终端"
else
    echo -e "  ${YELLOW}窗口0 (lidar)${NC}            - 激光雷达驱动"
    echo -e "  ${YELLOW}窗口1 (control)${NC}          - 底盘控制"
    echo -e "  ${YELLOW}窗口2 (decision_bt)${NC}      - 行为树总控(含定位/通信/视觉/执行器)"
    echo -e "  ${YELLOW}窗口3/4 (terminal_1/2)${NC}   - 调试终端"
fi
echo ""
echo -e "tmux 常用命令:"
echo -e "  ${YELLOW}进入会话:${NC} tmux attach -t $SESSION_NAME"
echo -e "  ${YELLOW}切换窗口:${NC} Ctrl+B 然后按窗口编号"
echo -e "  ${YELLOW}退出会话:${NC} Ctrl+B 然后按 d (detach)"
echo -e "  ${YELLOW}关闭会话:${NC} tmux kill-session -t $SESSION_NAME"
echo ""

read -r -p "是否立即进入tmux会话？[Y/n]: " attach_choice
case "$attach_choice" in
    [Nn]*)
        echo -e "${YELLOW}使用 'tmux attach -t $SESSION_NAME' 可随时进入会话${NC}"
        ;;
    *)
        tmux attach -t "$SESSION_NAME"
        ;;
esac
