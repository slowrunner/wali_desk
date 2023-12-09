#!/bin/bash


echo -e "\n*** GOTO -1,0 0deg"
echo -e 'ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true,goal_pose:{pose:{position:{x: -1,y: 0,z: 0.0}, orientation:{x: 0.0,y: 0.0, z: 0.0, w: 1.0}}}}"'
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true,goal_pose:{pose:{position:{x: -1,y: 0,z: 0.0}, orientation:{x: 0.0,y: 0.0, z: 0.0, w: 1.0}}}}"
