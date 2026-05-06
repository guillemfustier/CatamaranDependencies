from launch import LaunchDescription
from launch_ros.actions import Node


def _camera_node(name, device, topic, width=1920, height=1080, fps=30.0, quality=85):
    return Node(
        package='catamaran_camera_streams',
        executable='camera_compressed_publisher',
        name=name,
        output='screen',
        parameters=[{
            'camera_name': name,
            'video_device': device,
            'topic_name': topic,
            'frame_id': name,
            'image_width': width,
            'image_height': height,
            'fps': fps,
            'jpeg_quality': quality,
            'fourcc': 'MJPG',
        }],
    )


def generate_launch_description():
    return LaunchDescription([
        _camera_node(
            'front_fisheye_camera',
            '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:7:1.0-video-index0',
            '/catamaran/cameras/front_fisheye/compressed',
        ),
        _camera_node(
            'left_fisheye_camera',
            '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0',
            '/catamaran/cameras/left_fisheye/compressed',
        ),
        _camera_node(
            'back_fisheye_camera',
            '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0',
            '/catamaran/cameras/back_fisheye/compressed',
        ),
        _camera_node(
            'right_fisheye_camera',
            '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0',
            '/catamaran/cameras/right_fisheye/compressed',
        ),
        _camera_node(
            'logitech_camera',
            '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:6:1.0-video-index0',
            '/catamaran/cameras/logitech/compressed',
            width=640,
            height=480,
            quality=80,
        ),
    ])