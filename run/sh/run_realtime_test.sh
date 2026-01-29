#!/bin/bash

# 配置参数
CPU_ID=14
NODE_CMD="ros2 run jitter_analysis mock_robot_driver"

echo "==== 正在优化 CPU $CPU_ID 的实时环境 ===="

# 1. 尝试锁定 CPU 频率到性能模式 (需 root 权限)
if [ -f "/sys/devices/system/cpu/cpu$CPU_ID/cpufreq/scaling_governor" ]; then
    echo "performance" | sudo tee /sys/devices/system/cpu/cpu$CPU_ID/cpufreq/scaling_governor > /dev/null
    echo "[OK] CPU $CPU_ID 已设置为性能模式 (Performance)"
fi

# 2. 移除该核心的中断绑定 (手动调整 SMP Affinity)
# 将所有可移动中断从 CPU 14 移走，通常通过将掩码设为其他核实现
if [ -d "/proc/irq" ]; then
    echo "[INFO] 正在尝试降低 CPU $CPU_ID 的中断负载..."
    # 这里的操作比较复杂，通常关闭 irqbalance 即可
    sudo systemctl stop irqbalance 2>/dev/null
fi

# 3. 运行任务
# -c: 绑定核心
# chrt -f 99: 使用 SCHED_FIFO 实时调度策略，优先级设为最高 99
echo "[RUN] 正在以实时优先级启动任务..."
echo "------------------------------------------------"

sudo taskset -c $CPU_ID chrt -f 99 $NODE_CMD

# 退出后恢复 irqbalance (可选)
# sudo systemctl start irqbalance