#!/bin/bash

cd ~/wali_desk/config
wget https://raw.githubusercontent.com/iRobotEducation/create3_examples/asoragna/create3-republisher/create3_republisher/dds-config/fastdds-active-unicast.xml
mv fastdds-*-unicast.xml fastdds-desktop-unicast.xml
