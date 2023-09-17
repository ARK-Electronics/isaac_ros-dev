## Prerequisites
These instructions are based on kernel version 35.3.1 and Jetpack 5.1.1

If you have not already installed the OS with the custom device tree. Please see [Installing the OS from binaries](https://github.com/ARK-Electronics/ark_jetson_core#installing-the-os-from-binaries).

1. Install Jetpack:
```
sudo apt update
sudo apt install nvidia-jetpack
```

3. Restart the Docker service and add your user to the docker group:
```
sudo systemctl restart docker
sudo usermod -aG docker $USER
newgrp docker
```

4. Add default runtime in **/etc/docker/daemon.json**:
```
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
```
sudo systemctl daemon-reload && sudo systemctl restart docker
```

6. Install Git LFS in order to pull down all large files:
```
sudo apt-get install git-lfs
git lfs install --skip-repo
```

---

## Getting started with Isaac ROS2 on the Jetson

1. Clone the workspace:
```
mkdir -p  ~/workspaces/
git clone --recurse-submodules git@github.com:ARK-Electronics/isaac_ros-dev.git ~/workspaces/isaac_ros-dev/
```

2. Update your ~/.bashrc:
```
echo "export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev/" >> ~/.bashrc
source ~/.bashrc
```

3. Disable buildkit since it errors out. Open this file and add this line:
```
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts/
nano build_base_image.sh
```
``` diff
      print_warning "Building ${DOCKERFILE} as image: ${IMAGE_NAME} with base: ${BASE_IMAGE_NAME}"

+     DOCKER_BUILDKIT=0
      DOCKER_BUILDKIT=${DOCKER_BUILDKIT} docker build -f ${DOCKERFILE} \

```

4. Start the docker container for the first time. This will download a lot of images so be prepared to wait:
```
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && ./scripts/run_dev.sh
```

If all goes well, you should now find yourself inside of a docker container configured for development.
> admin@jake-nvidia:/workspaces/isaac_ros-dev$

## Running an example
If you've completed the steps above without errors, you're ready to run the first example.

1. Start the docker container if you haven't already:
```
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && ./scripts/run_dev.sh
```
Update package dependencies, rosdep will look for package.xml files in your workspace and install dependencies as needed using apt:
```
rosdep update
rosdep install --from-paths src/ --ignore-src -r -y
```
Build the packages and source the workspace, this may take some time:
```
cd /workspaces/isaac_ros-dev && \
  colcon build --symlink-install && \
  source install/setup.bash
```

2. (Optional) Run tests to verify complete and correct installation:
```
colcon test --executor sequential
```

3. Run the following launch files to spin up a demo of the isaac_ros_argus_camera package:
```
ros2 launch isaac_ros_argus_camera isaac_ros_argus_camera_mono.launch.py
```

## Testing a camera with gstreamer

If building the workspace was successful you can check the list of provided gstreamer elements from the **gst_bridge** package.
```
gst-inspect-1.0 install/
/lib/gst_bridge/librosgstbridge.so
```
> Plugin Details:
>   Name                     rosgstbridge
>   Description              ROS topic bridge elements
>   Filename                 install/gst_bridge/lib/gst_bridge/librosgstbridge.so
>   Version                  0.0.0
>   License                  LGPL
>   Source module            ros_gst_bridge
>   Binary package           ros_gst_bridge
>   Origin URL               https://github.com/BrettRD/ros-gst-bridge
>
>   rosaudiosink: rosaudiosink
>   rosimagesink: rosimagesink
>   rostextsink: rostextsink
>   rosaudiosrc: rosaudiosrc
>   rosimagesrc: rosimagesrc
>   rostextsrc: rostextsrc
>
>   6 features:
>   +-- 6 elements


gst-launch-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ rosimagesrc ros-topic="/left/image_raw" ! nvvidconv ! xvimagesink

---

### Issues

### Failed to create capture session
The Isaac ROS Argus node can fail to create a capture session inside the container after the nvargus daemon has crashed. By default, the  nvargus daemon is running in background, but it may crash due to other Argus clients. This will prevent Argus camera nodes from creating  capture sessions.
#### Solution
Exit the Docker container and restart the nvargus daemon:
```
sudo systemctl restart nvargus-daemon.service
```

### Failed to get calibration data from Argus!
If there is no available calibration data for an Argus camera, you will see warning messages similar to:
> WARN  extensions/hawk/argus_camera.cpp@677: Failed to get calibration data from Argus!
#### Solution
TODO