#/bin/bash

echo -e "\n*** ECHO ODOM"
echo -e "ros2 topic echo --once --flow-style /odom"
ros2 topic echo --once --flow-style -l 1 /odom

