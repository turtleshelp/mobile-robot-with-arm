# Copyright 2026 Muhammad Ali Sadiq (github.com/turtleshelp)
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='mobile_robot_perception',
            executable='cube_detector.py',
            name='cube_detector',
            output='screen',
        ),
    ])
