# Catamaran Dependencies

This repository holds the source code for key hardware control components of the Catamaran project. It includes ROS 2 packages for controlling the rudder motors and the SIYI ZR30 camera system.

## General Setup & Compilation

This repository is structured as a ROS 2 workspace. To build all the packages, navigate to the root of the repository and run:

```bash
colcon build
```

Remember to source the workspace after building:

```bash
source install/setup.bash
```

---

## Components

### 1. `dynamixel_sdk`

*   **Overview**: This is the official C++ SDK provided by ROBOTIS for controlling Dynamixel smart servos. It is included here as a fundamental dependency for the `motor_bringup` package. It provides the low-level communication protocol to read/write from the motor registers.

*   **Build**: This package is a C++ library and will be automatically built as a dependency when you run `colcon build` in the workspace root. You do not need to build it individually.

*   **Execution**: As a library, this package is not executed directly. It is used by the `rudder_angle_controller` node.

---

### 2. `motor_bringup`

*   **Overview**: This ROS 2 package is responsible for the high-level control of the catamaran's rudder, which is actuated by Dynamixel motors.

*   **Features**:
    *   **`rudder_angle_controller`**: A C++ ROS 2 node that subscribes to an angle topic. When it receives an angle, it calculates the corresponding position for the motor's rack and pinion system and commands the Dynamixel motor to move to that position.

*   **Build**: To build this package specifically, you can run:
    ```bash
    colcon build --packages-select motor_bringup
    ```

*   **Execution**:
    1.  Ensure your Dynamixel motors are connected and powered, and the `U2D2` or other interface is correctly configured (e.g., permissions for `/dev/ttyUSB0`).
    2.  In a sourced terminal, run the controller node:
        ```bash
        ros2 run motor_bringup rudder_angle_controller
        ```
    3.  To move the rudder, you can publish a message to its topic (the exact topic name is defined in the source code, but assuming it's `/rudder_angle`):
        ```bash
        # Example: Move rudder to 15.5 degrees
        ros2 topic pub /rudder_angle std_msgs/msg/Float32 "{data: 15.5}"
        ```

---

### 3. `siyi_camera_controller`

*   **Overview**: This package contains the `zr30camera` ROS 2 package, designed to interface with the SIYI ZR30 gimbal camera. It provides control over the gimbal's orientation, zoom, and focus. It includes the underlying Python SDK and ROS 2 nodes to expose its functionality.

*   **Dependencies**: This package has Python dependencies. Install them before running:
    ```bash
    pip install -r siyi_camera_controller/requirements.txt
    ```

*   **Configuration**: You need to follow the instructions in the [siyi_camera_controller README](./siyi_camera_controller/README.md) to configure the camera connection parameters.

*   **Features**:
    *   **`siyi_sdk`**: A Python library that handles the serial communication protocol for the SIYI camera.
    *   **`camera_controller`**: The main ROS 2 node that exposes topics and services for controlling the camera.
    *   **`cam_teleop`**: A simple Python script that provides keyboard-based teleoperation to control the gimbal and zoom for quick testing.

*   **Build**: This is a Python-based ROS 2 package. It will be correctly set up by the main `colcon build` command. To build it specifically:
    ```bash
    colcon build --packages-select zr30camera
    ```

*   **Execution**:
    1.  The primary way to run the camera system is using its launch file, which starts all necessary nodes.
        ```bash
        ros2 launch zr30camera zr30camera_launch.py
        ```
    2.  To manually control the camera from your keyboard for testing, open a new, sourced terminal and run the teleop script:
        ```bash
        ros2 run zr30camera cam_teleop
        ```

For more general information on the camera controller, see the [siyi_camera_controller README](./siyi_camera_controller/README.md).
