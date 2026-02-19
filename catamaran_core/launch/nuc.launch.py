import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

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
            # 2. Publicar GPS (Ublox + NTRIP) - NUC
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
            # 3. Cámara Siyi - NUC
            # ------------------------------------
            Node(
                package='zr30camera',
                executable='controller',
                name='zr30_controller',
                output='screen'
            ),
            Node(
                package='zr30camera',
                executable='stream',
                name='zr30_stream',
                output='screen',
                remappings=[
                    ('/ZR30/camera_stream', 'siyi_camera/image_raw') # Relativo: /catamaran/siyi_camera/...
                ]
            ),

            # ------------------------------------
            # 4. Cámara USB: - NUC
            # ------------------------------------
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name='usb_cam_node',
                parameters=[{
                    'video_device': '/dev/video0',
                    'framerate': 30.0,
                    'pixel_format': 'mjpeg2rgb',
                    'image_width': 640,
                    'image_height': 480,
                    'frame_id': 'usb_cam_optical_frame'
                }],
                remappings=[
                    ('image_raw', 'underwater_camera/image_raw'), # Relativo: /catamaran/underwater_camera/...
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        catamaran_ns
    ])
