#!/bin/bash

# REQ: sudo apt install ros-humble-image-view

ros2 run image_view disparity_view --ros-args --remap image:=/oak/stereo/image_raw
