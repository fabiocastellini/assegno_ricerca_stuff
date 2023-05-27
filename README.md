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

git clone https://github.com/creminem94/Weed-Removal-Robot.git
#mkdir -p ~/colcon_ws/src

# Rviz2:
cd ~/colcon_ws/src
git clone https://github.com/ros2/rviz.git --branch foxy
```


