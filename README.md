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
3. Install the relevant drivers for the ZED2i camera and fisheye camera as per the instructions in this [aoc_instruction4sensor-drivers](https://github.com/Cyano0/aoc_instruction4sensor-drivers.git) repository.

## RUN

### RGB-D cameras
#### Use real camera
1. Start the RGB-D camera node.
   
For Zed cameras, open a bash terminal and use the CLI command: 
```bash
$ ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>
```
Replace `<camera_model>` with the model of the camera that you are using: `zed`, `zedm`, `zed2`, `zed2i`, `zedx`, `zedxm`.

2. To launch the human detection and tracking node for the RGB-D camera, open a new bash terminal and use the CLI command: 
```bash
$ ros2 launch human_detection detection_rgbd.launch.xml
```
**NOTE**: Remember to check and update the argument ``image_topic `` in ``detection_rgbd.launch.xml`` files.

Other optional launch files include tracker_depth.launch.xml (for rgbd human detection with mask), tracker_depth_with_skeleton.launch.xml (for rgbd human detection with pose detection).

#### Use Rosbag data for testing

Rosbag data can be used for testing.
1. Run the ros bag provided in folder 'rosbag2_test'.
   
```
$ ros2 bag play rosbag2_test
```
Or replace rosbag's name `rosbag2_test` with the any other rosbags you have.

2. To launch the human detection and tracking node for the RGB-D camera, open a new bash terminal and use the CLI command: 
```bash
$ ros2 launch human_detection detection_rgbd.launch.xml
```
**NOTE**: Remember to check and update the argument ``image_topic `` to '/zed/zed_node/rgb_raw/image_raw_color' if using `rosbag2_test`.

#### Lauching detection for multiple Zed cameras

1. launch multiple cameras.
2. Launch the detection node for multiple cameras:
   ```
   $ ros2 launch human_detection launch_multi_zed_detection.py
   ```

### Fisheye cameras
1. To start the fisheye camera node, open a bash terminal and use the CLI command:
```bash
$ ros2 launch human_detection fisheye_camera_launch.xml
```
2. To launch the human detection and tracking node for the fisheye camera, open a new bash terminal and use the CLI command: 

```bash
$ ros2 launch human_detection detection_fisheye.launch.xml


```

