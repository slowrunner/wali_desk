#!/bin/bash

echo -e "\n** UNDOCK **"
echo -e '** ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"'
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
