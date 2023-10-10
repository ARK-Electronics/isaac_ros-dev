## Running argus_camera node with gstreamer

1. Start the docker container:
    ```bash
    ./run_docker.sh
    ```
    Build the packages and source the workspace:
    ```bash
    cd /workspaces/isaac_ros-dev && \
    colcon build --symlink-install && \
    source install/setup.bash
    ```
2. Run the following launch files to spin up a demo of the isaac_ros_argus_camera package:
    ```bash
    ros2 launch isaac_ros_argus_camera isaac_ros_argus_camera_mono.launch.py
    ```
3. Start the gstreamer pipelines. Replace `udpsink host=192.168.0.16` with your development machines IP. This assumes
both the development machine and Jetson are connected on the same network. <br>

    *Host*
    ```bash
    gst-launch-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ rosimagesrc ros-topic="/image_raw" ! queue max-size-buffers=1 ! videoconvert ! "video/x-raw,format=I420" ! x264enc bitrate=2000 tune=zerolatency speed-preset=ultrafast ! "video/x-h264,stream-format=byte-stream" ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.0.16 port=5600 sync=false
    ```
    *Client*
    ```bash
    gst-launch-1.0 udpsrc port=5600 ! application/x-rtp ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink

    ```
