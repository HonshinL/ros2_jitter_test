import matplotlib.pyplot as plt
import sys

def plot_cyclictest_hist(filename):
    latencies = []
    counts = []
    
    print(f"Reading {filename}...")
    try:
        with open(filename, 'r') as f:
            for line in f:
                # 跳过空行或注释
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                
                parts = line.split()
                # 核心改进：确保这一行只有两个元素，且都是数字
                if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
                    latencies.append(int(parts[0]))
                    counts.append(int(parts[1]))
    except Exception as e:
        print(f"Error reading file: {e}")
        return

    if not latencies:
        print("No valid data found in the file! Check if the file format is correct.")
        return

    # 2. 绘图
    plt.figure(figsize=(12, 7))
    plt.bar(latencies, counts, width=0.8, color='teal', alpha=0.8)
    
    plt.title(f'Latency Distribution - {filename}', fontsize=14)
    plt.xlabel('Latency (microseconds)', fontsize=12)
    plt.ylabel('Count (Log Scale)', fontsize=12)
    
    # 设置对数纵坐标，方便观察极少数的高延迟点
    plt.yscale('log') 
    plt.grid(True, which="both", ls="--", alpha=0.5)

    # 在图上标注最大延迟
    max_latency = latencies[-1] if latencies else 0
    plt.annotate(f'Max Latency: {max_latency}us', 
                 xy=(max_latency, counts[-1]), 
                 xytext=(max_latency*0.7, counts[-1]*10),
                 arrowprops=dict(facecolor='black', shrink=0.05))

    output_png = filename.replace('.txt', '.png')
    plt.savefig(output_png, dpi=300)
    print(f"Successfully saved plot to: {output_png}")

if __name__ == "__main__":
    file_to_read = sys.argv[1] if len(sys.argv) > 1 else 'output.txt'
    plot_cyclictest_hist(file_to_read)