from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Definir el namespace global
    catamaran_ns = GroupAction(
        actions=[
            PushRosNamespace('catamaran'),
            
            # ------------------------------------
            # 2. Publicar Orientation - Raspberry
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
            # 1. Puente MAVROS APM - Raspberry
            # ------------------------------------
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    FindPackageShare("mavros"), '/launch/apm.launch'
                ]),
                launch_arguments={
                    'fcu_url': 'udp://127.0.0.1:14550@'
                }.items()
            )
        ]
    )

    return LaunchDescription([
        catamaran_ns
    ])
