import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np
import time
import signal
import sys
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from collections import deque

# 全局变量
node = None

# 信号处理函数
def signal_handler(sig, frame):
    global node
    print("\n接收到退出信号，正在优雅退出...")
    if node:
        node.should_exit = True
    
    if rclpy.ok():
        rclpy.shutdown()
    
    plt.close('all')
    sys.exit(0)

class JitterAnalyzer(Node):
    def __init__(self, sampling_time=10.0, buffer_depth=3):
        super().__init__('jitter_analyzer')
        
        # 参数设置
        self.sampling_time = sampling_time
        self.start_time = None
        self.prev_real_time = None
        self.is_finished = False
        self.should_exit = False
        self.has_plotted = False
        self.creation_time = time.perf_counter()
        
        # FIFO缓冲区配置
        self.buffer_depth = buffer_depth  # 缓冲区深度（周期数）
        self.buffer = deque(maxlen=buffer_depth)  # 创建固定大小的缓冲区
        self.buffer_ready = False  # 标记缓冲区是否已填满
        
        # 预先分配内存
        self.max_samples = int(self.sampling_time * 150)
        self.intervals = np.zeros(self.max_samples, dtype=np.float64)
        self.interval_count = 0

        # 使用更高优先级的回调组
        self.callback_group = MutuallyExclusiveCallbackGroup()
        
        # 订阅机器人状态话题，使用BEST_EFFORT QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states', 
            self.listener_callback,
            qos_profile,
            callback_group=self.callback_group
        )

        # 创建固定频率的定时器来处理缓冲区数据
        self.target_period = 0.008  # 目标处理频率 125Hz (8ms)
        self.process_timer = self.create_timer(self.target_period, self.process_buffer)

        self.get_logger().info(f'已启动！将自动采集 {sampling_time} 秒数据...')
        self.get_logger().info(f'使用 {buffer_depth} 个周期的FIFO缓冲区，预计引入延迟: {buffer_depth*self.target_period*1000:.1f} ms')
        
        # 创建进度显示定时器
        self.create_timer(0.5, self.check_stop_condition)

    def listener_callback(self, msg):
        """将接收到的数据存入FIFO缓冲区"""
        if self.is_finished or self.should_exit:
            return

        # 将数据存入缓冲区
        self.buffer.append(msg)
        
        # 检查缓冲区是否已填满
        if len(self.buffer) == self.buffer_depth and not self.buffer_ready:
            self.buffer_ready = True
            self.get_logger().info('FIFO缓冲区已填满，开始处理数据...')

    def process_buffer(self):
        """以固定频率从缓冲区读取数据并处理"""
        if self.is_finished or self.should_exit or not self.buffer_ready:
            return

        current_real_time = time.perf_counter()
        
        if self.start_time is None:
            self.start_time = current_real_time
            self.prev_real_time = current_real_time
            return

        # 计算与上一帧的时间差 (ms)
        interval = (current_real_time - self.prev_real_time) * 1000
        
        # 存储数据
        if self.interval_count < self.max_samples:
            self.intervals[self.interval_count] = interval
            self.interval_count += 1
            
        self.prev_real_time = current_real_time

    def check_stop_condition(self):
        # 简单的进度显示
        if not self.is_finished and not self.should_exit:
            elapsed_time = time.perf_counter() - self.start_time if self.start_time else time.perf_counter() - self.creation_time
            if int(elapsed_time) % 1 == 0:
                print(f'\r采样进度: {min(elapsed_time, self.sampling_time):.1f}/{self.sampling_time}s', end='')
        
        stop_condition = self.should_exit or \
                        (self.start_time and (time.perf_counter() - self.start_time >= self.sampling_time)) or \
                        (not self.start_time and (time.perf_counter() - self.creation_time >= self.sampling_time))
        
        if stop_condition:
            if not self.is_finished:
                print()
                self.get_logger().info('采样完成，正在生成分析报告...')
                self.is_finished = True

    def plot_results(self):
        if self.interval_count == 0:
            self.get_logger().error('未采集到数据，请检查 /joint_states 话题是否在发布内容！')
            return

        # 提取有效数据
        data = self.intervals[:self.interval_count]
        if len(data) > 10:
            data = data[10:]  # 移除前10个不稳定样本

        # 解决中文显示问题
        import matplotlib
        matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'WenQuanYi Micro Hei', 'DejaVu Sans'] 
        matplotlib.rcParams['axes.unicode_minus'] = False 

        mean_v = np.mean(data)
        std_v = np.std(data)
        max_v = np.max(data)
        min_v = np.min(data)

        # 打印统计
        print(f"\n" + "="*50)
        print(f"采样时长: {self.sampling_time} s")
        print(f"FIFO缓冲区深度: {self.buffer_depth} 个周期")
        print(f"预计固定延迟: {self.buffer_depth*self.target_period*1000:.1f} ms")
        print(f"总样本数: {len(data)}")
        print(f"平均间隔: {mean_v:.4f} ms")
        print(f"标准差(抖动): {std_v:.4f} ms")
        print(f"最大延时: {max_v:.4f} ms")
        print(f"最小延时: {min_v:.4f} ms")
        print("="*50)

        # 检查是否需要退出
        if self.should_exit:
            return

        # 绘图
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        # 1. 频率分布直方图
        ax1.hist(data, bins=100, color='#3498db', edgecolor='white', alpha=0.8)
        ax1.axvline(8.0, color='#e74c3c', linestyle='--', label='目标周期 (8ms)')
        ax1.set_title(f'采样周期间隔分布 (FIFO缓冲区深度: {self.buffer_depth})')
        ax1.set_xlabel('时间间隔 (ms)')
        ax1.set_ylabel('频次')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # 2. 随时间变化的波动曲线
        ax2.plot(data, color='#2ecc71', linewidth=0.5)
        ax2.set_title('时间间隔波动趋势')
        ax2.set_xlabel('样本序号')
        ax2.set_ylabel('间隔 (ms)')
        ax2.set_ylim(0, 16)  # 聚焦在 0-16ms 观察
        ax2.axhline(8.0, color='#e74c3c', linestyle='--')
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        
        # 检查是否需要退出
        if self.should_exit:
            plt.close(fig)
            return
        
        # 使用阻塞方式显示图表
        plt.show()

def main():
    global node
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.init()
        # 可调整缓冲区深度：buffer_depth=2, 3 或 4
        node = JitterAnalyzer(sampling_time=10.0, buffer_depth=3)
        
        # 记录程序创建时间
        node.creation_time = time.perf_counter()
        
        # 使用单线程方式运行，确保GUI在主线程中执行
        while rclpy.ok() and not node.should_exit and not node.is_finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        # 在主线程中调用绘图函数
        if not node.should_exit and not node.has_plotted and node.interval_count > 0:
            node.plot_results()
            node.has_plotted = True
            
    except KeyboardInterrupt:
        print("\n接收到键盘中断，正在退出...")
    except Exception as e:
        print(f"\n发生错误: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()