#!/bin/bash

# Note: not implemented in simulator, docs say "implemented in G.3.1"

echo -e '\n** Fix Odom Drift with reset_pose 0,0/0'

echo -e '** CURRENT ODOM'
ros2 topic echo --once --flow-style /odom

echo -e '** SEND RESET_POSE'
ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose "{}"

echo -e '** New Odom'
ros2 topic echo --once --flow-style /odom
echo -e '********\n'
