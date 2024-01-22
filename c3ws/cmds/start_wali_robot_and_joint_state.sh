#!/bin/bash

# FILE:  start_wali_robot_and_joint_state.sh

echo -e "\n*** Starting Robot_State and Joint_State Publishers"
echo "*** with URDF file: create3.urdf.xacro"
ros2 launch wali wali_state_and_joint.launch.py 
