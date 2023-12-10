#!/bin/bash

echo -e "\n** DOCK for Galactic only **"
echo -e '** ros2 action send_goal /dock irobot_create_msgs/action/DockServo "{}"'
ros2 action send_goal /dock irobot_create_msgs/action/DockServo "{}"
