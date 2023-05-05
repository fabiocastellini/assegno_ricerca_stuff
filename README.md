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
- Install VSCode for programming + easy docker access: https://code.visualstudio.com/

### Run Docker container:
```
docker run --name r2_foxy -it osrf/ros:foxy-desktop
```

### Enable ROS2 GUI:
- Launch Xlaunch, select "DISPLAY NUMBER" (0), next, next, finish  
```
docker run --name r2_foxy -e DISPLAY=host.docker.internal:0.0 -it osrf/ros:foxy-desktop
```
