# task2.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='com2009_teamxx_2026',
            executable='task2.py',
            name='obstacle_avoidance',
            output='screen'
        )
    ])
