taskset -c 24 ros2 run jitter_analysis mock_robot_driver

taskset -c 15 ros2 run jitter_analysis jitter_analyzer_10s

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
git config http.version HTTP/1.1
