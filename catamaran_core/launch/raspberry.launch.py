from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    
    # Definir el namespace global
    catamaran_ns = GroupAction(
        actions=[
            PushRosNamespace('catamaran'),

            # ------------------------------------
            # 1. Publicar Orientation - Raspberry
            # ------------------------------------
            # Nodo MAVLink Bridge
            Node(
                package='catamaran_sensors',
                executable='mavlink_bridge',
                name='mavlink_bridge',
                output='screen',
                parameters=[{
                    'mavlink_ip': '0.0.0.0',  
                    'mavlink_port': 14550
                }]
            ),

            # Nodo TF GPS Publisher
            Node(
                package='catamaran_sensors',
                executable='tf_gps_publisher',
                name='tf_gps_publisher',
                output='screen',
                parameters=[{
                    'origin_lat': 39.99446582,
                    'origin_lon': -0.07405792
                }]
            ),

            # ------------------------------------
            # 2. CMD Vel Translator - Raspberry
            # ------------------------------------
            Node(
                package='motor_bringup',
                executable='cmdvel_motor_controller',
                name='cmdvel_motor_controller',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        catamaran_ns
    ])
