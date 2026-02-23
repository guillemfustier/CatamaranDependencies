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
            # # ------------------------------------
            # # 5. Cámara Siyi - NUC
            # # ------------------------------------
            # Node(
            #     package='zr30camera',
            #     executable='controller',
            #     name='zr30_controller',
            #     output='screen'
            # ),
            # Node(
            #     package='zr30camera',
            #     executable='stream',
            #     name='zr30_stream',
            #     output='screen',
            #     remappings=[
            #         ('/ZR30/camera_stream', 'siyi_camera/image_raw') # Relativo: /catamaran/siyi_camera/...
            #     ]
            # ),

            # ------------------------------------
            # 6. Cámaras USB: - NUC
            # ------------------------------------
            # Underwater Camera - exploreHD (/dev/video2)
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name='usb_cam_node_underwater',
                parameters=[{
                    'video_device': '/dev/video2',
                    'framerate': 30.0,
                    'pixel_format': 'mjpeg',
                    'image_width': 640,
                    'image_height': 480,
                    'frame_id': 'usb_cam_optical_frame'
                }],
                remappings=[
                    ('image_raw', 'underwater_camera/image_raw'), 
                ],
                output='screen'
            ),

            # Rear Camera - C270 (/dev/video6)
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name='usb_cam_node_rear',
                parameters=[{
                    'video_device': '/dev/video6',
                    'framerate': 30.0,
                    'pixel_format': 'yuyv',
                    'image_width': 640,
                    'image_height': 480,
                    'frame_id': 'rear_camera_optical_frame'
                }],
                remappings=[
                    ('image_raw', 'rear_camera/image_raw'), 
                ],
                output='screen'
            ),

            # Front Camera - exploreHD #2 (/dev/video4)
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name='usb_cam_node_front',
                parameters=[{
                    'video_device': '/dev/video4',
                    'framerate': 30.0,
                    'pixel_format': 'mjpeg',
                    'image_width': 640,
                    'image_height': 480,
                    'frame_id': 'front_camera_optical_frame'
                }],
                remappings=[
                    ('image_raw', 'front_camera/image_raw'), 
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        catamaran_ns
    ])
