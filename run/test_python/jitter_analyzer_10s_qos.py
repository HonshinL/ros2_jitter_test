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

# 全局变量
node = None

# 信号处理函数
def signal_handler(sig, frame):
    global node
    print("\n接收到退出信号，正在优雅退出...")
    if node:
        node.should_exit = True
    
    # 直接关闭ROS节点
    if rclpy.ok():
        rclpy.shutdown()
    
    # 关闭所有matplotlib窗口
    plt.close('all')
    
    # 强制退出程序
    sys.exit(0)

class JitterAnalyzer(Node):
    def __init__(self, sampling_time=10.0):
        super().__init__('jitter_analyzer')
        
        # 参数设置
        self.sampling_time = sampling_time
        self.start_time = None
        self.prev_real_time = None
        self.is_finished = False
        self.should_exit = False
        self.has_plotted = False
        self.creation_time = time.perf_counter()
        
        # 预先分配内存
        self.max_samples = int(self.sampling_time * 150)  # 假设最大频率150Hz
        self.intervals = np.zeros(self.max_samples, dtype=np.float64)
        self.interval_count = 0

        # 使用更高优先级的回调组
        self.callback_group = MutuallyExclusiveCallbackGroup()
        
        # 优化的QoS配置，适合实时抖动分析（兼容ROS 2 Humble版本）
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 优先考虑实时性而非可靠性
            durability=DurabilityPolicy.VOLATILE,       # 只关注最新数据
            history=HistoryPolicy.KEEP_LAST,            # 只保留最新消息
            depth=1                                     # 只保留1条历史消息
        )
        
        # 订阅机器人状态话题
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states', 
            self.listener_callback,
            qos_profile,
            callback_group=self.callback_group
        )

        self.get_logger().info(f'已启动！将自动采集 {sampling_time} 秒数据...')
        
        # 创建定时器，降低频率
        self.create_timer(0.5, self.check_stop_condition)

    def listener_callback(self, msg):
        if self.is_finished or self.should_exit:
            return

        current_real_time = time.perf_counter()  # 使用高精度计时器
        
        if self.start_time is None:
            self.start_time = current_real_time
            self.prev_real_time = current_real_time
            return

        # 计算与上一帧的时间差 (ms)
        interval = (current_real_time - self.prev_real_time) * 1000
        
        # 存储数据，避免动态分配
        if self.interval_count < self.max_samples:
            self.intervals[self.interval_count] = interval
            self.interval_count += 1
            
        self.prev_real_time = current_real_time

    def check_stop_condition(self):
        # 简单的进度显示，每秒更新一次
        if not self.is_finished and not self.should_exit:
            elapsed_time = time.perf_counter() - self.start_time if self.start_time else time.perf_counter() - self.creation_time
            if int(elapsed_time) % 1 == 0:  # 每秒打印一次
                print(f'\r采样进度: {min(elapsed_time, self.sampling_time):.1f}/{self.sampling_time}s', end='')
        
        stop_condition = self.should_exit or \
                        (self.start_time and (time.perf_counter() - self.start_time >= self.sampling_time)) or \
                        (not self.start_time and (time.perf_counter() - self.creation_time >= self.sampling_time))
        
        if stop_condition:
            if not self.is_finished:
                print()  # 换行
                self.get_logger().info('采样完成，正在生成分析报告...')
                self.is_finished = True
                
                # 如果是用户中断，不绘制图表，直接退出
                if self.should_exit:
                    self.get_logger().info('用户中断，正在退出...')
                    return

    def plot_results(self):
        if self.interval_count == 0:
            self.get_logger().error('未采集到数据，请检查 /joint_states 话题是否在发布内容！')
            return

        # 提取有效数据
        data = self.intervals[:self.interval_count]
        if len(data) > 10:
            data = data[10:]  # 移除前10个不稳定样本

        # --- 解决中文显示问题 ---
        import matplotlib
        matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'WenQuanYi Micro Hei', 'DejaVu Sans'] 
        matplotlib.rcParams['axes.unicode_minus'] = False 
        # ----------------------------

        mean_v = np.mean(data)
        std_v = np.std(data)
        max_v = np.max(data)
        min_v = np.min(data)

        # 打印统计
        print(f"\n" + "="*30)
        print(f"采样时长: {self.sampling_time} s")
        print(f"总样本数: {len(data)}")
        print(f"平均间隔: {mean_v:.4f} ms")
        print(f"标准差(抖动): {std_v:.4f} ms")
        print(f"最大延时: {max_v:.4f} ms")
        print(f"最小延时: {min_v:.4f} ms")
        print("="*30)

        # 检查是否需要退出
        if self.should_exit:
            return

        # 绘图
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        # 1. 频率分布直方图
        ax1.hist(data, bins=100, color='#3498db', edgecolor='white', alpha=0.8)
        ax1.axvline(8.0, color='#e74c3c', linestyle='--', label='目标周期 (8ms)')
        ax1.set_title('采样周期间隔分布 (Histogram)')
        ax1.set_xlabel('时间间隔 (ms)')
        ax1.set_ylabel('频次')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # 2. 随时间变化的波动曲线
        ax2.plot(data, color='#2ecc71', linewidth=0.5)
        ax2.set_title('时间间隔波动趋势 (Time Series)')
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
        node = JitterAnalyzer(sampling_time=10.0)  # 此处修改时长
        
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
        # 处理键盘中断
        print("\n接收到键盘中断，正在退出...")
    except Exception as e:
        print(f"\n发生错误: {e}")
    finally:
        # 确保资源正确释放
        if rclpy.ok():
            rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()