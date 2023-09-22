## Prerequisites
These instructions are based on kernel version 35.3.1 and Jetpack 5.1.1

If you have not already installed the OS with the custom device tree. Please see [Installing the OS from binaries](https://github.com/ARK-Electronics/ark_jetson_core#installing-the-os-from-binaries).

1. Install Jetpack:
    ```bash
    sudo apt update
    sudo apt install nvidia-jetpack
    ```

3. Restart the Docker service and add your user to the docker group:
    ```bash
    sudo systemctl restart docker
    sudo usermod -aG docker $USER
    newgrp docker
    ```

4. Add default runtime in **/etc/docker/daemon.json**:
    ```bash
    sudo apt install nano
    sudo nano /etc/docker/daemon.json
    ```
    Insert the following:
    ```diff
    {
        "runtimes": {
            "nvidia": {
                "path": "nvidia-container-runtime",
                "runtimeArgs": []
            }
        },
    +   "default-runtime": "nvidia"
    }
    ```

5. Restart docker:
    ```bash
    sudo systemctl daemon-reload && sudo systemctl restart docker
    ```

6. Install Git LFS in order to pull down all large files:
    ```bash
    sudo apt-get install git-lfs
    git lfs install --skip-repo
    ```

## Getting started with Isaac ROS2 on the Jetson

1. Clone the workspace:
    ```bash
    mkdir -p  ~/workspaces/
    git clone --recurse-submodules git@github.com:ARK-Electronics/isaac_ros-dev.git ~/workspaces/isaac_ros-dev/
    ```

2. Update your ~/.bashrc:
    ```bash
    echo "export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev/" >> ~/.bashrc
    source ~/.bashrc
    ```

3. Start the docker container for the first time. This will download a lot of images so be prepared to wait:
    ```bash
    ./run_docker.sh
    ```

If all goes well, you should now find yourself inside of a docker container configured for development.
> admin@jake-nvidia:/workspaces/isaac_ros-dev$

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

## Isaac ROS RealSense Setup
1. Clone the `librealsense` repo setup udev rules. Remove any connected RealSense cameras when prompted:
    ```bash
    cd /tmp && \
    git clone https://github.com/IntelRealSense/librealsense && \
    cd librealsense && \
    ./scripts/setup_udev_rules.sh
    ```

2. Clone the `4.51.1` release of the `realsense-ros` repository:

    ```bash
    cd ${ISAAC_ROS_WS}/src
    git clone https://github.com/IntelRealSense/realsense-ros.git -b 4.51.1
    ```

3. Plug in your RealSense camera before launching the docker container in the next step.

4. Add the realsense Dockerfile to the Docker file tree:
    ```bash
    nano .isaac_ros_common-config
    ```
    ```diff
    -CONFIG_IMAGE_KEY="ros2_humble.user.ark"
    +CONFIG_IMAGE_KEY="ros2_humble.user.ark.realsense"
    CONFIG_DOCKER_SEARCH_DIRS=($HOME/workspaces/isaac_ros-dev)
    ```

5. Launch the Docker container to rebuild the container image using `Dockerfile.realsense` in one of its layered stage. It will take some time for rebuilding.

    ```bash
    ./run_docker.sh
    ```

6. Before trying to use the realsense, ensure it's firmware is `5.13.0.50`. You can update the firmware by following the [Realsense documentation](https://dev.intelrealsense.com/docs/firmware-update-tool). You must use USB3 to update the realsense firmware. USB3 is also reccomended for operation due to the increased data rate.

7. Once container image is rebuilt and you are inside the container, you can run `realsense-viewer` to check that the RealSense camera is connected.

   ```bash
   realsense-viewer
   ```

   If you turn on the "Stereo Module" in the GUI, you should see something like the following:

## Troubleshooting

### Failed to create capture session
The Isaac ROS Argus node can fail to create a capture session inside the container after the nvargus daemon has crashed. By default, the  nvargus daemon is running in background, but it may crash due to other Argus clients. This will prevent Argus camera nodes from creating  capture sessions.
#### Solution
Exit the Docker container and restart the nvargus daemon:
```bash
sudo systemctl restart nvargus-daemon.service
```

### Failed to get calibration data from Argus!
If there is no available calibration data for an Argus camera, you will see warning messages similar to:
> WARN  extensions/hawk/argus_camera.cpp@677: Failed to get calibration data from Argus!
#### Solution
TODO