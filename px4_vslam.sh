#!/bin/bash
echo "Starting px4_vslam  Node"
source install/setup.bash
ros2 run px4_vslam vio_transform

