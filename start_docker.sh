#!/bin/bash

# Setup ROS2 environment
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

# Restart udev daemon
echo "restarting udev.. why tho"
sudo service udev restart

# Source setup so we can ros2 immediately
source install/setup.bash

# do nothing
tail -f