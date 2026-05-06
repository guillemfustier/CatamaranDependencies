# catamaran_camera_streams

Package that publishes the five camera feeds as `sensor_msgs/msg/CompressedImage` topics.

Stable device names are expected through `udev` symlinks:

- `/dev/catamaran_fisheye_front`
- `/dev/catamaran_fisheye_left`
- `/dev/catamaran_fisheye_back`
- `/dev/catamaran_fisheye_right`
- `/dev/catamaran_logitech`

Install the example rule file from `udev/99-catamaran-camera.rules.example` into `/etc/udev/rules.d/` and replace the serial placeholders with the real camera serials.

Launch:

```bash
ros2 launch catamaran_camera_streams camera_streams.launch.py
```
