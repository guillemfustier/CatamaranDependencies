from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_bringup',
            executable='cable_distance_controller',
            name='cable_distance_controller',
            parameters=[
                {'device_name': '/dev/ttyUSB0'},
                {'goal_topic': '/cable_distance'},
                {'actual_topic': '/cable_distance_actual'},
                {'initial_cable_distance': 1.0},
                {'max_cable_distance': 190.0},
            ]
        )
    ])