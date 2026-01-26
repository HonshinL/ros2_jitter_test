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

def plot_jitter_analysis(analysis_result, show_plot=True, save_plot=True, sampling_time=10.0):
    """可视化抖动分析结果"""
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
    
    # 创建图表
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # 1. 频率分布直方图 - 设置 align='mid' 实现居中显示
    ax1.hist(data, bins=100, color='#3498db', edgecolor='white', alpha=0.8, align='mid')
    ax1.axvline(8.0, color='#e74c3c', linestyle='--', label='目标周期 (8ms)')
    ax1.axvline(mean_v, color='#2ecc71', linestyle='-.', label=f'实际平均值 ({mean_v:.4f} ms)')
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
    
    # 保存图片
    if save_plot:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"jitter_analysis_{timestamp}.png"
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"\n图表已保存为: {filename}")
    
    # 显示图表
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
    
    # 可视化数据
    plot_jitter_analysis(analysis_result, show_plot=True, save_plot=True)

if __name__ == '__main__':
    main()