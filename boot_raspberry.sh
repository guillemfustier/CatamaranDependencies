#!/bin/bash

source /opt/ros/humble/setup.bash

cd CatamaranDependencies
source install/setup.bash


ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14551@



ros2 launch catamaran_sensors mavlink_tf.launch.py