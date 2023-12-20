#!/bin/bash

echo -e "\n** ECHO IR SENSOR VALUES"
echo -e "** ros2 topic echo --once  /ir_intensity | grep value"
echo -e "- side_left"
echo -e "- left"
echo -e "- front_left"
echo -e "- front_center_left"
echo -e "- front_center_right"
echo -e "- front_right"
echo -e "- right\n"
ros2 topic echo --once  /ir_intensity | grep value

