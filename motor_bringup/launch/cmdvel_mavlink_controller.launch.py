from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_bringup',
            executable='cmdvel_mavlink_controller',
            name='cmdvel_mavlink_controller',
            parameters=[
                {'udp_target_ip': '192.168.1.40'},  # Cambiar a la IP del autopiloto
                {'udp_target_port': 14550},
                {'target_system_id': 1},
                {'target_component_id': 1}
            ],
            output='screen'
        )
    ])
