# task1.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='com2009_teamxx_2026',
            executable='task1.py',
            name='circleof8',
            output='screen'
        )
    ])
