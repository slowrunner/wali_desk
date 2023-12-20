#!/bin/bash

echo -e "\n*** DRIVE 1cm ***"
echo -e '** ODOM Before'
ros2 topic echo --once --flow-style /odom

echo -e 'ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.010,max_translation_speed: 0.05}"'
ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.010,max_translation_speed: 0.05}"

echo -e '** ODOM After'
ros2 topic echo --once --flow-style /odom
