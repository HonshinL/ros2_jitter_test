import matplotlib.pyplot as plt
import numpy as np
import datetime

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'WenQuanYi Micro Hei', 'DejaVu Sans'] 
plt.rcParams['axes.unicode_minus'] = False 

def read_jitter_data(filename='jitter.txt'):
    """读取jitter.txt文件中的数据"""
    try:
        data = np.loadtxt(filename)
        print(f"成功读取 {filename} 文件，共 {len(data)} 个样本。")
        return data
    except FileNotFoundError:
        print(f"错误：无法找到文件 {filename}")
        return None
    except Exception as e:
        print(f"读取文件时发生错误：{e}")
        return None

def analyze_jitter_data(data):
    """分析抖动数据并返回统计信息"""
    if data is None or len(data) == 0:
        return None
    
    # 移除前10个不稳定样本
    if len(data) > 10:
        data = data[10:]
    
    # 计算统计信息
    mean_v = np.mean(data)
    std_v = np.std(data)
    max_v = np.max(data)
    min_v = np.min(data)
    
    return {
        'data': data,
        'mean': mean_v,
        'std': std_v,
        'max': max_v,
        'min': min_v,
        'sample_count': len(data)
    }

from scipy.stats import norm  # 新增：用于计算正态分布曲线

def plot_jitter_analysis(analysis_result, show_plot=True, save_plot=True, sampling_time=10.0, xlim=None):
    """可视化抖动分析结果（含拟合正态分布曲线）"""
    if analysis_result is None:
        print("没有可用的分析结果。")
        return
    
    data = analysis_result['data']
    mean_v = analysis_result['mean']
    std_v = analysis_result['std']
    max_v = analysis_result['max']
    min_v = analysis_result['min']
    sample_count = analysis_result['sample_count']
    
    # 打印统计信息
    print("\n" + "="*30)
    print(f"采样时长: {sampling_time} s")
    print(f"总样本数: {sample_count}")
    print(f"平均间隔: {mean_v:.4f} ms")
    print(f"标准差(抖动): {std_v:.4f} ms")
    print(f"最大延时: {max_v:.4f} ms")
    print(f"最小延时: {min_v:.4f} ms")
    print("="*30)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # --- 1. 频率分布直方图 + 正态分布曲线 ---
    # 修改点：density=True，将纵轴变为概率密度
    n, bins, patches = ax1.hist(data, bins=100, color='#3498db', edgecolor='white', 
                                alpha=0.6, align='mid', density=True, label='实际分布')
    
    # 计算正态分布曲线的 x 和 y
    # 根据数据范围生成 200 个平滑的点
    x_curve = np.linspace(min(data), max(data), 200)
    y_curve = norm.pdf(x_curve, mean_v, std_v)
    
    # 绘制拟合曲线
    ax1.plot(x_curve, y_curve, color='#e67e22', linewidth=2.5, label='正态分布拟合')
    
    # ax1.axvline(8.0, color='#2ecc71', linestyle='--', label='目标周期 (8ms)')
    ax1.axvline(mean_v, color='#e74c3c', linestyle='-.', label=f'实际平均值')
    
    ax1.set_title('采样周期间隔分布 (Histogram)')
    ax1.set_xlabel('时间间隔 (ms)')
    ax1.set_ylabel('概率密度')
    
    if xlim is not None:
        ax1.set_xlim(xlim)
    
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # --- 2. 随时间变化的波动曲线 (保持不变) ---
    ax2.plot(data, color='#2ecc71', linewidth=0.5)
    ax2.set_title('时间间隔波动趋势 (Time Series)')
    ax2.set_xlabel('样本序号')
    ax2.set_ylabel('间隔 (ms)')
    ax2.set_ylim(max(0, mean_v-5), mean_v+5) # 稍微动态调整范围
    ax2.axhline(8.0, color='#e74c3c', linestyle='--')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    
    if save_plot:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"jitter_fit_{timestamp}.png"
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"\n图表已保存为: {filename}")
    
    if show_plot:
        plt.show()
    else:
        plt.close(fig)

def main():
    """主函数"""
    print("抖动数据可视化工具")
    print("-" * 50)
    
    # 读取数据
    data = read_jitter_data('jitter.txt')
    if data is None:
        return
    
    # 分析数据
    analysis_result = analyze_jitter_data(data)
    if analysis_result is None:
        return
    
    # 可视化数据，设置x轴区间为[7.5, 8.5]，聚焦在目标周期附近
    plot_jitter_analysis(analysis_result, show_plot=True, save_plot=True, xlim=(0.0, 20.0))
    # plot_jitter_analysis(analysis_result, show_plot=True, save_plot=True)

if __name__ == '__main__':
    main()