#!/bin/bash

# 1. 加载系统环境变量（包含 ROS 2 底层路径）
source ~/.bashrc

# 2. 获取当前脚本所在的绝对路径，确保在任何地方执行都能找到 install 文件夹
SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
else
    echo "错误: 找不到 install/setup.bash，请确保在工作空间根目录运行此脚本。"
    exit 1
fi

# 3. 打印提示信息（可选，方便调试）
echo "正在核心 14 上启动 mock_robot_driver..."

# 4. 执行命令
# 使用 exec 可以让脚本进程被目标程序替换，方便管理信号
# ddsrouter -c $SCRIPT_DIR/client.yaml
taskset -c 14 ros2 run jitter_analysis mock_robot_driver