import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import random

class MockRobotDriver(Node):
    def __init__(self):
        super().__init__('mock_robot_driver')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        # 目标频率 125Hz (8ms)
        self.timer_period = 0.008  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info('模拟机器人驱动已启动，正在以 125Hz 发布数据...')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        msg.position = [0.0] * 6
        
        # --- 模拟系统抖动 (可选) ---
        # 如果你想模拟真实系统的抖动，可以取消下面这一行的注释：
        # time.sleep(random.uniform(0, 0.002)) 
        
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = MockRobotDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()