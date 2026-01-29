#!/usr/bin/env python3
"""
Launch file for running jitter analysis nodes.

This launch file starts both the mock robot driver and the jitter analyzer.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with multiple nodes."""
    # Create the mock robot driver node
    mock_driver_node = Node(
        package='jitter_analysis',
        executable='mock_robot_driver',
        name='mock_robot_driver',
        output='screen',
        parameters=[],
        remappings=[]
    )

    # Create the jitter analyzer node
    jitter_analyzer_node = Node(
        package='jitter_analysis',
        executable='jitter_analyzer_10s',
        name='jitter_analyzer',
        output='screen',
        parameters=[],
        remappings=[]
    )

    # Return the launch description with both nodes
    return LaunchDescription([
        mock_driver_node,
        jitter_analyzer_node
    ])