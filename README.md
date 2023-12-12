## Prerequisites
These instructions are based on kernel version 35.3.1 and Jetpack 5.1.1

If you have not already installed the OS with the custom device tree. Please see [Installing the OS from binaries](https://github.com/ARK-Electronics/ark_jetson_kernel#installing-the-os-from-binaries).

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
    -   }
    +   },
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

First ensure your github user credentials are set and your ssh public key has been added to your github account.

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

You are now ready to build all of the packages in this workspace. This will take a long time the first time you do it:
```
colcon build --symlink-install
```
Once the build is complete exit the docker container, sync the filesystem, and reboot the jetson.
```
exit
sync
sudo reboot
```

## Isaac ROS VSLAM with RealSense D455i and PX4
These are instructions for setting up the RealSense D455i to use as a full solution for VSLAM integrated with PX4

1. Clone the `librealsense` repo setup udev rules. Remove any connected RealSense cameras when prompted:
    ```bash
    cd /tmp && \
    git clone https://github.com/IntelRealSense/librealsense && \
    cd librealsense && \
    ./scripts/setup_udev_rules.sh
    ```

2. Ensure Realsense 455i firmware is `5.13.0.50`. You can update the firmware by following the [Realsense documentation](https://dev.intelrealsense.com/docs/firmware-update-tool). You must use USB3 to update the realsense firmware. USB3 is also reccomended for operation due to the increased data rate.

3. Copy the **ros-nodes.service** into systemd. This service will launch the **isaac_ros_visual_slam** realsense node, the foxglove bridge, a gstreamer pipeline, and a node to convert the VSLAM solution into the PX4 `vehicle_visual_odometry` message.
```
sudo cp ros-nodes.service /etc/systemd/system/ && sudo systemctl enable ros-nodes.service
```
4. Enable the service and reboot
```
sudo systemctl enable ros-nodes.service && sudo reboot
```