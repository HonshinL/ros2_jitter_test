#!/bin/bash

CPU_ID=14

echo "--- 1. 内核隔离参数检查 ---"
if grep -q "isolcpus=.*$CPU_ID" /proc/cmdline; then
    echo "[OK] 内核参数已配置 isolcpus 隔离 CPU $CPU_ID"
else
    echo "[WARN] 未发现 isolcpus 配置，CPU $CPU_ID 可能仍在处理系统调度"
fi

echo -e "\n--- 2. 实时负载检查 (Top Processes on CPU $CPU_ID) ---"
# 查看当前跑在 CPU 14 上的进程
ps -eMo pid,tid,psr,pcpu,comm | grep -E "^ *PID|^.* +$CPU_ID +" | head -n 10

echo -e "\n--- 3. 硬件中断 (Interrupts) 检查 ---"
# 检查是否有硬件中断绑定到了 CPU 14
INTR_COUNT=$(awk -v cpu=$CPU_ID '{print $(cpu+2)}' /proc/interrupts | awk '{sum += $1} END {print sum}')
echo "CPU $CPU_ID 上的总中断触发次数: $INTR_COUNT"

if [ "$INTR_COUNT" -gt 1000 ]; then
    echo "[WARN] 中断次数较高，建议检查 irqbalance 设置"
fi