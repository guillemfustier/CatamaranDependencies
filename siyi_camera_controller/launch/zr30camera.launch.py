from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zr30camera',
            executable='controller',
            name='zr30_controller'
        ),
        Node(
            package='zr30camera',
            executable='stream',
            name='zr30_stream'
        )
    ])