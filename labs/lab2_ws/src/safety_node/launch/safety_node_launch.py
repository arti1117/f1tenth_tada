from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safety_node',
            executable='safety_node.py',
            name='SafetyNode',
        )            
    ])