# strawberry_data_collection

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
2. In the `src` directory, clone this [strawberry_data_collection](https://github.com/Prabuddhi-05/strawberry_data_collection.git) repository:
```bash
$ cd ~/{ROS2_WORKSPACE}/src
$ git clone git@github.com:Prabuddhi-05/strawberry_data_collection.git
```
3. Install the relevant drivers for the ZED 2i camera and Arducam fisheye camera as per the instructions in this [aoc_instruction4sensor-drivers](https://github.com/Cyano0/aoc_instruction4sensor-drivers.git) repository.
   
4. Then build using colcon build in the current directory.
```bash
cd ~/{ROS2_WORKSPACE} && colcon build
source ~/.bashrc 
```
## RUN
1. To start the fisheye camera node, open a bash terminal and use the CLI command:
```bash
$ ros2 launch human_detection fisheye_camera_launch.xml
```
2. To launch the human detection and tracking node for the fisheye camera, open a new bash terminal and use the CLI command: 

```bash
$ ros2 launch human_detection detection_fisheye.launch.xml


```

