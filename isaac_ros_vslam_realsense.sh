#!/bin/bash
echo "Starting VSLAM RealSense Node"
source install/setup.bash
ros2 launch px4_vslam vslam_realsense.launch.py
