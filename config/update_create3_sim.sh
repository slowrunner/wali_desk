#/bin/bash

echo "UPDATE CREATE3 SIM"

echo "Pulling create3 sim repo changes to c3ws/src"
cd ~/wali_desk/c3ws/src/create3_sim
git pull

echo "Using vsc to update additional dependencies to workspace"
# Use vcs to update additional dependencies into the workspace (gazebo not on galactic yet)
cd ~/wali_desk/c3ws
vcs import ~/wali_desk/c3ws/src/ < ~/wali_desk/c3ws/src/create3_sim/dependencies.repos

echo "Checking for ROS2 dependencies"
cd ~/wali_desk/c3ws
rosdep install --from-path src -yi

echo "Building create3 sim"
colcon build --symlink-install

echo "Remember to source install/local_setup.bash"

