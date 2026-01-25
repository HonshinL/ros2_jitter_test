taskset -c 14 ros2 run jitter_analysis mock_robot_driver

taskset -c 15 ros2 run jitter_analysis jitter_analyzer_10s

ROS_DOMAIN_ID=10 taskset -c 15 ros2 run jitter_analysis jitter_analyzer_10s

source ~/.bashrc
source ./install/setup.bash
taskset -c 14 ros2 run jitter_analysis mock_robot_driver

echo $ROS_DOMAIN_ID && echo $RMW_IMPLEMENTATION && echo $ROS_LOCALHOST_ONLY

ddsrouter -c client.yaml

sudo ufw allow 11666/udp