#include <chrono>
#include <memory>
#include <random>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class MockRobotDriver : public rclcpp::Node
{
public:
  MockRobotDriver()
  : Node("mock_robot_driver")
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockRobotDriver>());
  rclcpp::shutdown();
  return 0;
}