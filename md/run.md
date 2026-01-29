taskset -c 24 ros2 run jitter_analysis mock_robot_driver

taskset -c 25 ros2 run jitter_analysis jitter_analyzer_10s

ROS_DOMAIN_ID=10 taskset -c 15 ros2 run jitter_analysis jitter_analyzer_10s

source ~/.bashrc
source ./install/setup.bash
taskset -c 14 ros2 run jitter_analysis mock_robot_driver

echo $ROS_DOMAIN_ID && echo $RMW_IMPLEMENTATION && echo $ROS_LOCALHOST_ONLY

ddsrouter -c client.yaml

sudo ufw allow 11666/udp
sudo ufw allow 7400:12000/udp
sudo ufw delete allow 7400:12000/udp
sudo ufw allow from 192.168.1.0/16 to any port 7400:12000 proto udp
sudo ufw allow from 224.0.0.0/4 to any port 7400:12000 proto udp
sudo ufw allow from 192.168.0.0/16 to any port 1024:65535 proto udp
sudo ufw delete 2

sudo apt install fastdds-tools

==============================
采样时长: 10 s
总样本数: 1301
平均间隔: 7.9997 ms
标准差(抖动): 0.0624 ms
最大延时: 8.3574 ms
最小延时: 7.7154 ms
==============================
==============================
采样时长: 10 s
总样本数: 1302
平均间隔: 7.9998 ms
标准差(抖动): 0.0415 ms
最大延时: 8.2168 ms
最小延时: 7.7784 ms
==============================
==============================
采样时长: 10 s
总样本数: 1302
平均间隔: 7.9998 ms
标准差(抖动): 0.0357 ms
最大延时: 8.1903 ms
最小延时: 7.7137 ms
==============================
==============================
采样时长: 10 s
总样本数: 1301
平均间隔: 7.9998 ms
标准差(抖动): 0.0258 ms
最大延时: 8.1425 ms
最小延时: 7.8721 ms
==============================
==============================
采样时长: 10 s
总样本数: 1301
平均间隔: 7.9998 ms
标准差(抖动): 0.0227 ms
最大延时: 8.1053 ms
最小延时: 7.8705 ms
==============================