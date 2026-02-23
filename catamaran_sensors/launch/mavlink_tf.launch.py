import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'origin_lat',
            default_value='39.99446582',
            description='Latitude origin'
        ),
        DeclareLaunchArgument(
            'origin_lon',
            default_value='-0.07405792',
            description='Longitude origin'
        ),

        # Nodo MAVLink Bridge
        launch_ros.actions.Node(
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
        launch_ros.actions.Node(
            package='catamaran_sensors',
            executable='tf_gps_publisher',
            name='tf_gps_publisher',
            output='screen',
            parameters=[{
                'origin_lat': LaunchConfiguration('origin_lat'),
                'origin_lon': LaunchConfiguration('origin_lon')
            }]
        )
    ])

