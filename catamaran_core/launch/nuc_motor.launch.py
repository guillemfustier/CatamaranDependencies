import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share_sensors = get_package_share_directory('catamaran_sensors')
    config_file = os.path.join(pkg_share_sensors, 'config', 'ublox_gps.yaml')

    # Definir el namespace global
    catamaran_ns = GroupAction(
        actions=[
            PushRosNamespace('catamaran'),

            # ------------------------------------
            # 1. Timón (Dynamixel) - NUC
            # ------------------------------------
            Node(
                package='motor_bringup',
                executable='rudder_angle_controller_abs',
                name='rudder_angle_controller_abs',
                output='screen'
            ),

            # ------------------------------------
            # 2. CMD Vel Translator - NUC
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
