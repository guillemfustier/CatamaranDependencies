import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('catamaran_sensors')
    config_file = os.path.join(pkg_share, 'config', 'ublox_gps.yaml')

    return LaunchDescription([
        # 1. Driver Node (Talks to USB)
        Node(
            package='ublox_dgnss_node',
            executable='ublox_dgnss_node',
            name='ublox_dgnss',
            output='screen',
            parameters=[config_file]
        ),
        
        # 2. Translator Node (UBX -> NavSatFix)
        Node(
            package='ublox_nav_sat_fix_hp_node',
            executable='ublox_nav_sat_fix_hp',
            name='ublox_nav_sat_fix_hp',
            output='screen',
            remappings=[
                ('fix', '/mavlink/gps')
            ]
        ),

        # 3. NTRIP Client (RTK Corrections from Internet)
        Node(
            package='ntrip_client_node',
            executable='ntrip_client_node',
            name='ntrip_client',
            output='screen',
            parameters=[{
                'host': 'icverva2.icv.gva.es',   # Servidor de Valencia
                'port': 2101,                    # Puerto estandar NTRIP
                'mountpoint': 'BORR3',           # El de Borriana que es el más cercano a nosotros
                'username': 'uni13n5',           # Tu usuario del email
                'password': 'uni13n5',           # Tu contraseña del email
                'use_https': False,              # Normalmente NTRIP va por HTTP puro
                'log_level': 'debug'           # Descomentar si no conecta
            }]
        )
    ])