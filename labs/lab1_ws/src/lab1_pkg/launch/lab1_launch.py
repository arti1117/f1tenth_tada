from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Talker node (publishes to 'drive')
        Node(
            package='lab1_pkg',
            executable='talker',
            name='talker',
            parameters=[{'v':1.5, 'd':0.3}] # Set parameters here
        ),

        Node(
            package='lab1_pkg',
            executable='relay.py',
            name='relay',
        )
    ])