#include <chrono>
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip> // 用于设置输出精度
#include <csignal> // 用于信号处理
#include <fstream> // 用于文件操作

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

namespace jitter_analysis
{
  bool should_exit = false;

  void signal_handler(int sig)
  {
    std::cout << "\n接收到退出信号，正在优雅退出..." << std::endl;
    should_exit = true;
  }

  class JitterAnalyzer : public rclcpp::Node
  {
  public:
    explicit JitterAnalyzer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
        double sampling_time = 10.0)
        : Node("jitter_analyzer", options),
          sampling_time_(sampling_time),
          is_finished_(false),
          interval_count_(0)
    {
      // 预先分配内存
      max_samples_ = static_cast<size_t>(sampling_time_ * 150); // 假设最大频率150Hz
      intervals_.resize(max_samples_, 0.0);

      // 使用更快的消息接收方式
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states",
          rclcpp::SensorDataQoS(), // 使用传感器数据QoS，更适合高频数据
          std::bind(&JitterAnalyzer::listener_callback, this, std::placeholders::_1));

      // 创建定时器检查停止条件（500ms间隔）
      timer_ = this->create_wall_timer(1000ms, std::bind(&JitterAnalyzer::check_stop_condition, this));

      RCLCPP_INFO(this->get_logger(), "已启动！将自动采集 %.1f 秒数据...", sampling_time_);
      creation_time_ = std::chrono::high_resolution_clock::now();
    }

    // 组件模式下不需要run函数，因为容器会管理节点的生命周期
    // 所有处理都在回调函数中完成
    void run()
    {
      RCLCPP_WARN(this->get_logger(), "run() function is not used in component mode");
    }

  private:
    // 在listener_callback中添加超时检查
    void listener_callback(const sensor_msgs::msg::JointState::SharedPtr /*msg*/)
    {
      if (is_finished_ || should_exit)
        return;

      auto current_real_time = std::chrono::high_resolution_clock::now();

      // 计算与上一帧的时间差 (ms)
      if (start_time_ != std::chrono::time_point<std::chrono::high_resolution_clock>())
      {
        auto interval = std::chrono::duration<double, std::milli>(
                            current_real_time - prev_real_time_)
                            .count();

        // 存储数据
        if (interval_count_ < max_samples_)
        {
          intervals_[interval_count_] = interval;
          interval_count_++;
        }
      }
      else
      {
        start_time_ = current_real_time;
      }

      prev_real_time_ = current_real_time;

      // 直接检查是否完成采样
      auto elapsed_time = std::chrono::duration<double>(
                              current_real_time - start_time_)
                              .count();

      if (elapsed_time >= sampling_time_ && !is_finished_)
      {
        std::cout << std::endl;
        RCLCPP_INFO(this->get_logger(), "采样完成，正在生成分析报告...");
        is_finished_ = true;

        // 立即执行分析和保存
        if (interval_count_ > 0)
        {
          analyze_results();
          save_data_to_file();
        }
      }
      else if (!is_finished_ && !should_exit)
      {
        // 只更新进度显示，不执行其他操作
        std::cout << "\r采样进度: " << std::min(elapsed_time, sampling_time_) << "/" << sampling_time_ << "s" << std::flush;
      }
    }

    // 简化定时器回调
    void check_stop_condition()
    {
      // 只检查外部退出信号
      if (should_exit && !is_finished_)
      {
        std::cout << std::endl;
        RCLCPP_INFO(this->get_logger(), "接收到退出信号，正在退出...");
        is_finished_ = true;
      }
    }

    void analyze_results()
    {
      if (interval_count_ == 0)
      {
        RCLCPP_ERROR(this->get_logger(), "未采集到数据，请检查 /joint_states 话题是否在发布内容！");
        return;
      }

      // 提取有效数据，移除前10个不稳定样本
      size_t start_index = interval_count_ > 10 ? 10 : 0;
      size_t data_size = interval_count_ - start_index;

      double mean_v = 0.0;
      double std_v = 0.0;
      double max_v = intervals_[start_index];
      double min_v = intervals_[start_index];

      // 计算均值
      for (size_t i = start_index; i < interval_count_; i++)
      {
        mean_v += intervals_[i];
        if (intervals_[i] > max_v)
          max_v = intervals_[i];
        if (intervals_[i] < min_v)
          min_v = intervals_[i];
      }
      mean_v /= data_size;

      // 计算标准差
      for (size_t i = start_index; i < interval_count_; i++)
      {
        double diff = intervals_[i] - mean_v;
        std_v += diff * diff;
      }
      std_v = std::sqrt(std_v / data_size);

      // 打印统计信息
      std::cout << "\n"
                << "==============================" << std::endl;
      std::cout << "采样时长: " << sampling_time_ << " s" << std::endl;
      std::cout << "总样本数: " << data_size << std::endl;
      std::cout << "平均间隔: " << std::fixed << std::setprecision(4) << mean_v << " ms" << std::endl;
      std::cout << "标准差(抖动): " << std::fixed << std::setprecision(4) << std_v << " ms" << std::endl;
      std::cout << "最大延时: " << std::fixed << std::setprecision(4) << max_v << " ms" << std::endl;
      std::cout << "最小延时: " << std::fixed << std::setprecision(4) << min_v << " ms" << std::endl;
      std::cout << "==============================" << std::endl;
    }

    void save_data_to_file()
    {
      if (interval_count_ == 0)
      {
        std::cout << "没有数据需要保存。" << std::endl;
        return;
      }

      // 打开文件
      std::ofstream outfile("jitter.txt");
      if (!outfile.is_open())
      {
        std::cerr << "无法打开文件 jitter.txt 进行写入。" << std::endl;
        return;
      }

      // 写入数据
      std::cout << "\n正在保存数据到 jitter.txt..." << std::endl;
      for (size_t i = 0; i < interval_count_; i++)
      {
        outfile << std::fixed << std::setprecision(6) << intervals_[i] << std::endl;
      }

      // 关闭文件
      outfile.close();
      std::cout << "数据保存完成，共 " << interval_count_ << " 个样本。" << std::endl;
    }

    double sampling_time_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
    std::chrono::time_point<std::chrono::high_resolution_clock> prev_real_time_;
    std::chrono::time_point<std::chrono::high_resolution_clock> creation_time_;

    bool is_finished_;
    size_t max_samples_;
    std::vector<double> intervals_;
    size_t interval_count_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
  };

  // Register the component
  RCLCPP_COMPONENTS_REGISTER_NODE(jitter_analysis::JitterAnalyzer)
} // namespace jitter_analysis

// Component library does not need main function - remove it