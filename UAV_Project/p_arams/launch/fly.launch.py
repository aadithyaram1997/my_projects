#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p_arams',
            node_executable='project_script',
            node_name='project_script',
            output='screen'),        
    ])