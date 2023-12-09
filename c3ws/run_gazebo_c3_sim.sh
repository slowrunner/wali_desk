#!/bin/bash

echo "Starting Create3 Sim in Gazebo"
cd ~/wali_desk/c3ws
source install/local_setup.bash

ros2 launch irobot_create_gazebo create3.launch.py
