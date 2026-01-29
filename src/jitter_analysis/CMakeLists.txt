cmake_minimum_required(VERSION 3.8)
project(jitter_analysis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 添加Python和NumPy依赖
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
include_directories(
  include
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)

# Create component library
add_library(jitter_analysis_components SHARED
  src/mock_robot_driver_comp.cpp
  src/jitter_analyzer_10s_comp.cpp
)

# Set component compile options
target_compile_features(jitter_analysis_components PUBLIC c_std_99 cxx_std_17)
target_include_directories(jitter_analysis_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add component dependencies
ament_target_dependencies(jitter_analysis_components
  rclcpp
  rclcpp_components
  sensor_msgs
)

# Register component nodes
rclcpp_components_register_nodes(jitter_analysis_components "jitter_analysis::MockRobotDriver")
rclcpp_components_register_nodes(jitter_analysis_components "jitter_analysis::JitterAnalyzer")

# Install component library
install(TARGETS
  jitter_analysis_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Keep the original executables for standalone execution (optional)
add_executable(mock_robot_driver src/mock_robot_driver.cpp)
ament_target_dependencies(mock_robot_driver rclcpp sensor_msgs)

add_executable(jitter_analyzer_10s src/jitter_analyzer_10s.cpp)
ament_target_dependencies(jitter_analyzer_10s rclcpp sensor_msgs)

install(TARGETS
  mock_robot_driver
  jitter_analyzer_10s
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()