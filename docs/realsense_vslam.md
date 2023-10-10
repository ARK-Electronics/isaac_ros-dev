## RealSense D455i Setup
1. Clone the `librealsense` repo setup udev rules. Remove any connected RealSense cameras when prompted:
    ```bash
    cd /tmp && \
    git clone https://github.com/IntelRealSense/librealsense && \
    cd librealsense && \
    ./scripts/setup_udev_rules.sh
    ```

2. Ensure Realsense 455i firmware is `5.13.0.50`. You can update the firmware by following the [Realsense documentation](https://dev.intelrealsense.com/docs/firmware-update-tool). You must use USB3 to update the realsense firmware. USB3 is also reccomended for operation due to the increased data rate.

## Realsense and VSLAM
1. Build the workspace inside of the docker container. This assumes you have already setup the docker environment in the [README](../README.md)
	```
	colcon build && source install/setup.bash
	```

2. You should now be able to launch the realsense node with Isaac ROS VSLAM:

   ```bash
   ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
   ```

3. In another terminal attached to the docker container, check that the camera topics are being published:
	```
	source install/setup.bash
	ros2 topic list
	```
4. If you see the `/camera/infra1/image_rect_raw` you can view it in QGC with a gstreamer pipeline:
	```
	TARGET_IP=192.168.40.4
	TARGET_PORT=5600
	gst-launch-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ rosimagesrc ros-topic=/camera/infra1/image_rect_raw ! queue max-size-buffers=1 ! video/x-raw,format=GRAY8 ! videoconvert ! x264enc bitrate=2000 tune=zerolatency speed-preset=ultrafast ! video/x-h264,stream-format=byte-stream ! rtph264pay config-interval=1 pt=96 ! udpsink host=$TARGET_IP port=$TARGET_PORT sync=false
	```

