import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np
import time
import signal
import sys
from tqdm import tqdm

# 全局变量用于优雅退出
should_exit = False

# 信号处理函数
def signal_handler(sig, frame):
    global should_exit
    print("\n接收到退出信号，正在优雅退出...")
    should_exit = True
    
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
        self.intervals = []
        self.prev_real_time = None
        self.is_finished = False
        self.progress_bar = None
        self.has_plotted = False  # 新增：标记是否已经绘制过图表

        # 订阅机器人状态话题
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states', 
            self.listener_callback,
            10)

        self.get_logger().info(f'已启动！将自动采集 {sampling_time} 秒数据...')
        
        # 创建一个定时器来检查是否到达停止时间
        self.create_timer(0.1, self.check_stop_condition)
        
        # 立即初始化进度条
        self.progress_bar = tqdm(total=self.sampling_time, unit='s', desc='采样进度', ncols=80)

    def listener_callback(self, msg):
        if self.is_finished or should_exit:
            return

        current_real_time = time.perf_counter() # 使用高精度计时器
        
        if self.start_time is None:
            self.start_time = current_real_time
            self.prev_real_time = current_real_time
            return

        # 计算与上一帧的时间差 (ms)
        interval = (current_real_time - self.prev_real_time) * 1000
        self.intervals.append(interval)
        self.prev_real_time = current_real_time

    def check_stop_condition(self):
        global should_exit
        
        # 无论是否收到消息，都更新进度条
        if self.progress_bar and not self.is_finished and not should_exit:
            elapsed_time = time.perf_counter() - self.start_time if self.start_time else time.perf_counter() - self.creation_time
            self.progress_bar.n = min(elapsed_time, self.sampling_time)
            self.progress_bar.refresh()
        
        if should_exit or (self.start_time and (time.perf_counter() - self.start_time >= self.sampling_time)) or (not self.start_time and (time.perf_counter() - self.creation_time >= self.sampling_time)):
            if not self.is_finished:
                self.get_logger().info('采样完成，正在生成分析报告...')
                self.is_finished = True
                
                # 关闭进度条
                if self.progress_bar:
                    self.progress_bar.close()
                
                # 如果是用户中断，不绘制图表，直接退出
                if should_exit:
                    self.get_logger().info('用户中断，正在退出...')
                    return
                
                # 绘制图表并标记为已绘制
                if not self.has_plotted:
                    self.plot_results()
                    self.has_plotted = True

    def plot_results(self):
        if not self.intervals or should_exit:
            return

        # --- 新增：解决中文显示问题 ---
        import matplotlib
        # 设置中文字体（Linux 下常用 'WenQuanYi Micro Hei' 或 'Droid Sans Fallback'）
        # Windows 下常用 'SimHei'
        matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'WenQuanYi Micro Hei', 'DejaVu Sans'] 
        matplotlib.rcParams['axes.unicode_minus'] = False # 解决负号显示问题
        # ----------------------------

        data = np.array(self.intervals)
        data = data[10:]
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
        if should_exit:
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
        ax2.set_ylim(0, 16) # 聚焦在 0-16ms 观察
        ax2.axhline(8.0, color='#e74c3c', linestyle='--')
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        
        # 检查是否需要退出
        if should_exit:
            plt.close(fig)
            return
        
        # 使用阻塞方式显示图表
        plt.show()

def main():
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.init()
        node = JitterAnalyzer(sampling_time=10.0) # 此处修改时长
        
        # 记录程序创建时间
        node.creation_time = time.perf_counter()
        
        # 使用循环替代rclpy.spin()，确保能检查退出条件
        while rclpy.ok() and not should_exit and not node.is_finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        # 确保资源正确释放
        if node.progress_bar:
            node.progress_bar.close()
        
        # 移除重复绘制图表的代码
        # if not should_exit and node.intervals:
        #     node.plot_results()
            
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