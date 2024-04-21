#!/bin/bash

if [ $ROS_DISTRO == "galactic" ]; then
    source /opt/ros/galactic/setup.bash
  else
    source /opt/ros/humble/setup.bash
fi

source ~/wali_desk/c3ws/install/setup.bash

# export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/wali_desk/config/super_client_configuration_file.xml
# export ROS_DISCOVERY_SERVER=10.0.0.219:11811

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/wali_desk/config/fastdds-desktop-unicast.xml
