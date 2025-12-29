#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    my_robot_slam = get_package_share_directory('my_robot_slam')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_robot_slam, 'launch', 'localization.launch.py')
            ),
        ),
        Node(
            package='my_robot_slam',
            executable='coverage',
            name='coverage',
            output='screen'),
        Node(
            package='my_robot_slam',
            executable='set_initial_pose',
            name='set_initial_pose',
            output='screen'),
    ])
