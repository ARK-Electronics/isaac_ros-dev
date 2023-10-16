#!/bin/bash
echo "Starting VSLAM RealSense Node"
source install/setup.bash
ros2 launch imu_transform jake_vslam_realsense.launch.py
