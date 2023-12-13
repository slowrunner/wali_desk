#!/bin/bash

echo -e "\n*** ECHO DOCK STATUS"
if [ $ROS_DISTRO == "galactic" ]
  then
    echo -e "ros2 topic echo /dock_status"
    ros2 topic echo /dock_status
  else
    echo -e "ros2 topic echo --once /dock_status"
    ros2 topic echo --once /dock_status
fi
