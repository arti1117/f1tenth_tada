from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab1_pkg',
            executable='talker',
            name='talker',
            parameters=[{'v': 1.2, 'd': 0.4}]
        )
    ])
