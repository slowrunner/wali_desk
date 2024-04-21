#!/bin/bash

# REQ: sudo apt install ros-humble-image-view

ros2 run image_view image_view --ros-args --remap image:=/oak/rgb/image_raw
