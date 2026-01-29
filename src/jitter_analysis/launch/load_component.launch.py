#!/usr/bin/env python3
"""
Launch file for running jitter analysis components in a container.

This launch file starts a component container and loads both the mock robot driver
and the jitter analyzer components into it.
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with component container."""
    # Create the component container
    container = ComposableNodeContainer(
        name='jitter_analysis_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Add the mock robot driver component
            ComposableNode(
                package='jitter_analysis',
                plugin='jitter_analysis::MockRobotDriver',
                name='mock_robot_driver'),
            # Add the jitter analyzer component
            ComposableNode(
                package='jitter_analysis',
                plugin='jitter_analysis::JitterAnalyzer',
                name='jitter_analyzer')
        ],
        output='screen',
    )

    # Return the launch description
    return LaunchDescription([container])