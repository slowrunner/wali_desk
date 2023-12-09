#!/bin/bash

echo -e "\n*** UNDOCK "
echo -e "*** DOCK STATUS"
echo -e "ros2 echo topic --once /dock_status"
ros2 echo topic --once /dock_status
echo -e '*** ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"'
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"


