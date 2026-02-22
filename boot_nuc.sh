#!/bin/bash

source /opt/ros/humble/setup.bash

cd Catamarandependencies
source install/setup.bash

ros2 launch catamaran_core nuc.launch.py