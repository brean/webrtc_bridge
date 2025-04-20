#! /usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='webrtc_bridge',
            executable='webrtc_bridge',
            name='webrtc_bridge'
        )
    ])
