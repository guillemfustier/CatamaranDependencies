# Catamaran Dependencies

This repository is a ROS 2 workspace for Catamaran hardware and perception dependencies. It includes packages for motor control, the SIYI ZR30 camera, onboard camera streams, and camera republishing for the GUI.

## Setup

Import the external repositories, if they are not already present:

```bash
vcs import . < catamaran_dependencies.repos
```

Build the workspace from the repository root:

```bash
colcon build
```

Source the workspace after building:

```bash
source install/setup.bash
```

## External Repositories

Some dependencies are kept as standalone repositories and are listed in [`catamaran_dependencies.repos`](./catamaran_dependencies.repos):

- [`unilidar_sdk`](https://github.com/unitreerobotics/unilidar_sdk.git)
- [`SurroundViewRealTimeCatamaran`](https://github.com/JosepMarinG/SurroundViewRealTimeCatamaran.git)

## Documented Packages

This README is kept as a compact entry point. Package-specific setup, launch commands, topics, and troubleshooting notes live in each package README.

### `motor_bringup`

ROS 2 package for catamaran motor control. The current package README documents the MAVLink-based `cmdvel_mavlink_controller`, the legacy Mavros launch option, build command, parameters, and troubleshooting notes.

See the [`motor_bringup` README](./motor_bringup/README.md).

---

### `siyi_camera_controller`

ROS 2 Humble driver for the SIYI ZR30 camera. It includes the Python SDK wrapper and nodes for camera stream, zoom, gimbal, and focus control.

See the [`siyi_camera_controller` README](./siyi_camera_controller/README.md) for dependencies, network setup, launch command, topics, and teleoperation.

---

### `catamaran_camera_streams`

Publishes the catamaran's five onboard camera feeds as `sensor_msgs/msg/CompressedImage` topics.

See the [`catamaran_camera_streams` README](./catamaran_camera_streams/README.md) for the expected camera device symlinks, `udev` rule setup, and launch command.

---

### `catamaran_congestion_control`

Subscribes to the compressed camera topics and Unitree lidar cloud, applies per-stream FPS and quality controls, and republishes adapted data for the MetaQuest GUI.

See the [`catamaran_congestion_control` README](./catamaran_congestion_control/README.md) for the config file, run command, and control topic details.

---

### `dynamixel_sdk`

Official ROBOTIS C++ SDK for Dynamixel smart servos. It is included as a dependency for motor-related packages and is built automatically with the workspace.
