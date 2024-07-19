# fisheye_camera_driver

This repository contains a ROS2 package for launching a ZED camera and a fisheye camera to capture images of strawberries in a polytunnel and record the data in ROS 2 bags.

## SETUP 

### PREREQUISITES:
- Ubuntu 22.04
- ROS2 Humble

### Git clone repository
1. Create a ROS2 workspace and a source directory (`src`):
```bash
$ mkdir -p ~/{ROS2_WORKSPACE}/src
```
2. In the `src` directory, clone this [fisheye_camera_driver](https://github.com/Prabuddhi-05/fisheye_camera_driver.git) repository:
```bash
$ cd ~/{ROS2_WORKSPACE}/src
$ git clone git@github.com:Prabuddhi-05/fisheye_camera_driver.git
```
3. Install the relevant drivers for the Arducam fisheye camera as per the instructions in this [aoc_instruction4sensor-drivers](https://github.com/Cyano0/aoc_instruction4sensor-drivers.git) repository.
   
4. Then build using colcon build in the current directory.
```bash
cd ~/{ROS2_WORKSPACE} && colcon build
source ~/.bashrc 
```
## RUN
To launch the fisheye camera node, open a terminal and use the following command:
```bash
$ ros2 launch fisheye_camera_driver multiple_fisheye_launch.xml
```  

