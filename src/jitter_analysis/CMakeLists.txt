cmake_minimum_required(VERSION 3.8)
project(jitter_analysis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 添加Python和NumPy依赖
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
include_directories(
  include
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)

# mock robot driver
add_executable(mock_robot_driver src/mock_robot_driver.cpp)
ament_target_dependencies(mock_robot_driver rclcpp sensor_msgs)

# jitter analyzer (without matplotlib)
add_executable(jitter_analyzer_10s src/jitter_analyzer_10s.cpp)
ament_target_dependencies(jitter_analyzer_10s rclcpp sensor_msgs)

install(TARGETS
  mock_robot_driver
  jitter_analyzer_10s
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()