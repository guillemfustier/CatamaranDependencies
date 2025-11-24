from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Motor nodes
        Node(
            package='motor_bringup',
            executable='cmdvel_motor_controller',
            name='cmdvel_motor_controller'
        ),
        Node(
            package='motor_bringup',
            executable='rudder_angle_controller_abs',
            name='rudder_angle_controller_abs'
        ),
        # Camera nodes
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