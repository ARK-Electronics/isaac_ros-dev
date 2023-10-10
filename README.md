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

You are now ready to build all of the packages in this workspace. This will take a long time the first time you do it:
```
colcon build --symlink-install
```

#### For more tutorials see the [docs](docs/) folder