# assegno_ricerca_stuff
This repo contains useful data/markdowns/improvized doc relative to my research project (2023) about precision agriculture.

# Notes about ROS2 foxy Docker
### Sources:
> https://www.youtube.com/watch?v=qWuudNxFGOQ&t=1s&ab_channel=MuhammadLuqman
> https://github.com/noshluk2/ros1_wiki/blob/main/docker/commands.md
> https://docs.ros.org/en/foxy/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html

### Installation:
- Install docker: https://www.docker.com/
- Get ROS2 foxy image: docker pull osrf/ros:foxy-desktop
- Install Xlaunch (required for GUI): https://sourceforge.net/projects/xming/
- Install VSCode for programming + easy docker access with Docker extension: https://code.visualstudio.com/
- To reinstall RVIZ2 in case of "abort error":
```
cd ~/rviz2_ws/src
git clone -b foxy https://github.com/ros2/rviz.git
cd ../
colcon build --merge-install
```

### Run Docker container:
```
docker run --name r2_foxy -it osrf/ros:foxy-desktop
```


### Enable ROS2 GUI:
- Launch Xlaunch, select "DISPLAY NUMBER" (0), next, next, finish  
```
docker run --name r2_foxy -e DISPLAY=host.docker.internal:0.0 -it osrf/ros:foxy-desktop
docker run --name r2_foxy_slam -e DISPLAY=host.docker.internal:0.0 -it r2_foxy_slam:version1_05_05_23
```


### ROS1 Melodic with GUI for mmWave:
```
docker run --name r1_melodic -e DISPLAY=host.docker.internal:0.0 -it osrf/ros:melodic-desktop-bionic
 apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*
Link: https://blog.csdn.net/qq_37429313/article/details/124235487

```

### To create a new Docker image starting from another:
- Create folder in Desktop called Docker
- Create "Dockerfile"
```
FROM osrf/ros:foxy-desktop
RUN apt-get update
RUN sudo apt-get install git && apt-get install python3-pip
RUN sudo apt-get install terminator
RUN sudo apt install python3-colcon-common-extensions
RUN mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
RUN echo "ALL DONE"
```
- Go to Desktop/Docker folder
- Run:
```
 docker build -t r2_foxy_from_file .
```

#### Alternative: commit a container that is running
```
docker commit original_name new_name:tag_name
docker save -o r2_foxy_slam.tar r2_foxy_slam

docker run -it 
```

### To source the colon_ws
```
source install/local_setup.bash
source /opt/ros/foxy/setup.bash
source root/colcon_ws/install/local_setup.bash
export ROS_DOMAIN_ID=1
```

### Access docker container
docker exec -it 5899a125a370 /bin/bash

### ROBOT
 ssh agri@10.42.0.3
 
 
### Install ROS on EDATEC CM4 Nano
- flash Ubuntu 20.04 64 bit on the internal memory
- install ROS2 https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
- Run following commands
 ```
sudo apt-get install ros-foxy-sensor-msgs-py
sudo apt install python3-pip
pip install opencv-python
sudo apt install python3-pykdl 
pip install timm

git clone https://bitbucket.org/qbrobotics/qbdevice-api-6.x.x.git
cd qbdevice-api-6.x.x/src/ && make

# Doens't work cause it's a private folder! Mount 
and copy file
git clone https://github.com/creminem94/Weed-Removal-Robot.git
#mkdir -p ~/colcon_ws/src

# Rviz2: I solved doing colcon build on ros2_foxy folder!
cd ~/colcon_ws/src
git clone https://github.com/ros2/rviz.git --branch foxy
```

### Install Realsense on PC ROS2 & CM4 Nano 
Source: https://github.com/vanderbiltrobotics/realsense-ros
```
sudo apt-get install ros-foxy-realsense2-camera
ros2 launch realsense2_camera rs_launch.py
# always source envorionmnet:
source ~/ros2_ws/install/setup.bash
source /opt/ros/foxy/setup.bash

### To have multiple terminals while on SSH with CM4 Nano not working!:
Source: https://askubuntu.com/questions/332104/open-another-terminal-window-with-the-same-ssh-session-as-original-window
sudo apt install gnome-terminal

### To mount a USB drive and access data on raspy/CM4 nano
Source: https://www.youtube.com/watch?v=stqVx3uTBFA
sudo fdisk -l
sudo mount /dev/sda1 /media/usb
```

### mmWave Sensor (IWR6843LEVM Texas Instruments)
Launch online demo:
https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/3.6.0/

List USB ports:
ll /dev/serial/by-id

Modify serial ports in the launch file:
gedit ~/mmwave_ti_ros/ros_driver/src/ti_mmwave_rospkg/launch/...

Flash the board, follow this guide: https://www.ti.com/video/6205855073001
https://dev.ti.com/tirex/explore/node?node=A__AB268N4OBosFpbbKHivyuw__com.ti.mmwave_industrial_toolbox__VLyFKFf__4.9.0
- Download: https://www.ti.com/tool/UNIFLASH
```
sudo ln -sf /lib/x86_64-linux-gnu/libudev.so.1 /lib/x86_64-linux-gnu/libudev.so.0
cd ~/Downloads & chmod +x uniflash_sl.8.3.0.4307.run 
./uniflash_sl.8.3.0.4307.run 
Install required library:
sudo apt-get install libgconf-2-4
```

Install sdk from https://www.ti.com/tool/MMWAVE-SDK 
```
sudo apt-get install libc6:i386
cd ~/Downloads & chmod +x mmwave_sdk_03_06_00_00-LTS-Linux-x86-Install.bin 
./mmwave_sdk_03_06_00_00-LTS-Linux-x86-Install.bin

Tutorial: https://dev.ti.com/tirex/explore/node?a=VLyFKFf__4.11.0&node=A__ABfVSftqTn9gPnXnrqFxdg__com.ti.mmwave_industrial_toolbox__VLyFKFf__4.11.0
git clone https://git.ti.com/cgit/mmwave_radar/mmwave_ti_ros/
cd mmwave_ti_ros/autonomous_robotics_ros/ && catkin_make
source devel/setup.bash


https://github.com/NVIDIA/nvidia-docker/issues/1238
sudo apt-get install curl

distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

Running the Small Obstacle Detection demo:
- https://dev.ti.com/tirex/explore/content/radar_toolbox_1_00_01_07/tools/visualizers/Industrial_Visualizer/docs/mmWave_Industrial_Visualizer_User_Guide.html
- flashing the small_obstacle_detection_6843.bin file

Integrating mmWave into ROS2 starting from the GUI and having flashed the small obstacle detection demo:
```
ros2 pkg create --build-type ament_python ti_mmwave_ros2
```

# To play ROS2 bags:
sudo apt-get install ros-foxy-rosbag2-storage-default-plugins ros-foxy-ros2bag

### Install CUDA (gpu)
- Install conda: https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html
- Install Cuda drivers (https://pytorch.org/get-started/locally/): conda install pytorch torchvision torchaudio pytorch-cuda=11.7 -c pytorch -c nvidia

# MoveIt2 and UR5 installation on ROS2 Foxy
- Source https://moveit.ros.org/install-moveit2/source/
cd ros2_ws/src
git clone https://github.com/ros-planning/moveit2.git -b $ROS_DISTRO
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp





cd ros2_ws
git clone -b foxy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Trying out docker:
https://moveit.picknik.ai/main/doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu.html


# ROS2 ISSUES:
- SetuptoolsDeprecationWarning
pip install setuptools==58.2.0
- Remove "-" from setup.cfg and insert "_"

