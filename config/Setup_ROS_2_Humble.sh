#!/bin/bash

# (Headless) SETUP ROS2 GoPiGo3 (HUMBLE HAWKSBILL) ON Ubuntu 22.04 64-bit Server (Jammy Jellyfish)


# == Check for UTF-8 locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale

# == Ensure Ubuntu Universe repo is enabled
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# === Setup GPG key to ROS2
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# === Add ROS2 Repository to sources
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# === Make sure system is up to date before continuing
sudo apt update
sudo apt upgrade -y

# === Install ROS2 Humble Hawksbill desktop packages
sudo apt install -y ros-humble-desktop

# === Install ROS 2 Compilers and tools to build packages
sudo apt install -y ros-dev-tools

# === Source setup script
source /opt/ros/humble/setup.bash

# === Setup colcon build tool
sudo apt install -y python3-colcon-common-extensions

# === Setup ROS2 in user's .bashrc file
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# === Tell ROS2 to inhabit "Domain 0" 
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

# === Install build environment
# sudo apt install -y build-essential
# pip3 install setuptools==58.2.0

# (does transforms3d need sudo pip3 ?)
pip3 install transforms3d

# === Setup ROSDEP tool
sudo apt install -y python3-rosdep2
rosdep update

# ==== Create a ROS2 Workspace with source folder

mkdir -p ~/wali_desk/c3ws/src

=== Setup colcon_cd in .bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc

# === Setup colcon to know where workspace is
echo "export _colcon_cd_root=~/wali_desk/c3ws" >> ~/.bashrc

# setup to know about built packages 
echo "source ~/wali_desk/c3ws/install/setup.bash" >> ~/.bashrc

# === SETUP ROSDEP
sudo apt install -y python3-rosdep2
rosdep update


# === Build everything
. ~/.bashrc
cd ~/wali_desk/c3ws
# ./build_all.sh
# . install/setup.bash

echo -e "\n********** ROS2 HUMBLE READY FOR TEST ***********\n"
