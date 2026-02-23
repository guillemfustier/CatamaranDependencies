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
            # 3. Publicar GPS (Ublox + NTRIP) - NUC
            # ------------------------------------
            # Driver Node (Talks to USB)
            Node(
                package='ublox_dgnss_node',
                executable='ublox_dgnss_node',
                name='ublox_dgnss',
                output='screen',
                parameters=[config_file]
            ),
            
            # Translator Node (UBX -> NavSatFix)
            Node(
                package='ublox_nav_sat_fix_hp_node',
                executable='ublox_nav_sat_fix_hp',
                name='ublox_nav_sat_fix_hp',
                output='screen',
                remappings=[
                    ('fix', 'mavlink/gps')  # Relativo: /catamaran/mavlink/gps
                ]
            ),

            # NTRIP Client (RTK Corrections from Internet)
            Node(
                package='ntrip_client_node',
                executable='ntrip_client_node',
                name='ntrip_client',
                output='screen',
                parameters=[{
                    'host': 'icverva2.icv.gva.es',
                    'port': 2101,
                    'mountpoint': 'BORR3',
                    'username': 'uni13n5',
                    'password': 'uni13n5',
                    'use_https': False,
                    'log_level': 'debug'
                }]
            ),

            # ------------------------------------
            # 4. Robot Description - NUC
            # ------------------------------------
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("catamaran_description"), '/launch/robot_description.launch.py'
                ])
            )
        ]
    )

    return LaunchDescription([
        catamaran_ns
    ])
