#include <chrono>
#include <memory>
#include <random>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

namespace jitter_analysis
{
class MockRobotDriver : public rclcpp::Node
{
public:
  explicit MockRobotDriver(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("mock_robot_driver", options)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    timer_ = this->create_wall_timer(8ms, std::bind(&MockRobotDriver::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "模拟机器人驱动已启动，正在以 125Hz 发布数据...");
  }

private:
  void timer_callback()
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    msg.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // --- 模拟系统抖动 (可选) ---
    // 如果你想模拟真实系统的抖动，可以取消下面这一行的注释：
    // std::this_thread::sleep_for(std::chrono::microseconds(std::rand() % 2000)); 
    
    publisher_->publish(msg);
  }
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(jitter_analysis::MockRobotDriver)
}  // namespace jitter_analysis

// Component library does not need main function - remove it