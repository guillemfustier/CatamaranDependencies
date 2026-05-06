from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_bringup',
            executable='lars_height_controller',
            name='lars_height_controller',
            parameters=[
                {'device_name': '/dev/ttyUSB0'},
                {'goal_topic': '/lars_height'},
                {'actual_topic': '/lars_height_actual'},
                {'initial_height': 24.0},
                {'max_height': 24.0},
            ]
        )
    ])