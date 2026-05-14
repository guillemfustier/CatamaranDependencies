# catamaran_camera_republisher

ROS 2 Humble C++ node that subscribes to the compressed camera topics published by `catamaran_camera_streams`, receives per-camera control commands, and republishes adapted compressed images for the MetaQuest GUI.

## Configuration

Per-camera ROS topics and limits are defined in `config/catamaran_cameras.yaml` under each `cameras[]` entry (`input_topic`, `output_topic`, `control_topic`, FPS/quality defaults and bounds, and `quality_mode`). Point `config_file` at that path (or a copy you maintain) when launching the node.

## Run

```bash
ros2 run catamaran_camera_republisher adaptive_camera_republisher \
  --ros-args \
  -p config_file:=$(ros2 pkg prefix --share catamaran_camera_republisher)/config/catamaran_cameras.yaml
```

## Control topics

Each camera listens on its `control_topic` from the config for `std_msgs/msg/Int32MultiArray`:

```text
[fps, quality]
```

Examples (topic names match the default `front_fisheye` row in `catamaran_cameras.yaml`; use the `control_topic` value for other cameras):

```bash
ros2 topic pub /catamaran/cameras/front_fisheye/control std_msgs/msg/Int32MultiArray "{data: [10, 80]}" --once
ros2 topic pub /catamaran/cameras/front_fisheye/control std_msgs/msg/Int32MultiArray "{data: [0, 0]}" --once
```

