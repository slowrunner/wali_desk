#!/bin/bash

echo -e "\n** DOCK **"
echo -e '** ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"'
ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"
