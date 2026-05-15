# catamaran_camera_republisher

ROS 2 Humble C++ republishers that subscribe to camera and lidar topics, receive per-stream control commands, and republish adapted data for the MetaQuest GUI.

## Configuration

Per-camera ROS topics and limits are defined in `config/catamaran_cameras.yaml` under each `cameras[]` entry (`input_topic`, `output_topic`, `control_topic`, FPS/quality defaults and bounds, and `quality_mode`). Point `config_file` at that path (or a copy you maintain) when launching the node.

The lidar republisher uses `config/catamaran_lidar.yaml` under `streams[]`. Its default stream subscribes to `/unilidar/cloud`, publishes `/unilidar/cloud/republished`, and listens for control on `/unilidar/cloud/control`.

## Run

First make sure the camera streams launch is already running (and also the lidar)

```bash
ros2 launch catamaran_camera_streams camera_streams.launch.py
```

```bash
ros2 launch unitree_lidar_ros2 launch.py
```

Then launch the republishers:

```bash
ros2 run catamaran_camera_republisher adaptive_camera_republisher \
  --ros-args \
  -p config_file:=$(ros2 pkg prefix --share catamaran_camera_republisher)/config/catamaran_cameras.yaml
```

```bash
ros2 run catamaran_camera_republisher adaptive_lidar_republisher \
  --ros-args \
  -p config_file:=$(ros2 pkg prefix --share catamaran_camera_republisher)/config/catamaran_lidar.yaml
```

## Control topics

Each stream listens on its `control_topic` from the config for `std_msgs/msg/Int32MultiArray`:

```text
[fps, quality]
```

Examples (topic names match the default `front_fisheye` row in `catamaran_cameras.yaml`; use the `control_topic` value for other cameras):

```bash
ros2 topic pub /catamaran/cameras/front_fisheye/control std_msgs/msg/Int32MultiArray "{data: [10, 80]}" --once
ros2 topic pub /catamaran/cameras/front_fisheye/control std_msgs/msg/Int32MultiArray "{data: [0, 0]}" --once
ros2 topic pub /unilidar/cloud/control std_msgs/msg/Int32MultiArray "{data: [10, 50]}" --once
```

For cameras, `quality` controls resize and/or JPEG quality according to `quality_mode`. For lidar, `quality` is the percentage of points kept in the republished `PointCloud2`.
