import os
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
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
            # parameters=[{
            #     'origin_lat': 39.99446582,
            #     'origin_lon': -0.07405792
            # }]
        )
    ])

