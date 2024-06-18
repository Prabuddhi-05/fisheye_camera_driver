# strawberry_data_collection

This repository contains a ROS2 package for launching a ZED camera and a fisheye camera to capture images of strawberries in a polytunnel and record the data in ROS 2 bags.

## SETUP 

### PREREQUISITES:
- Ubuntu 22.04
- Ros2 Humble

### Git clone repositories
1. Create a ROS2 workspace and a source directory (`src`):
```bash
$ mkdir -p ~/{ROS2_WORKSPACE}/src
```
2. In the `src` directory, clone this [human_detection_rgbd_camera](https://github.com/LCAS/human_detection_rgbd_camera.git) repository:
```bash
$ cd ~/{ROS2_WORKSPACE}/src
$ git clone git@github.com:LCAS/human_detection_rgbd_camera.git
```
3. In the `src` directory, clone the [ultralytics_ros](https://github.com/Alpaca-zip/ultralytics_ros.git) repository. Further, install dependencies and build the workspace: 
```bash

$ cd ~/{ROS2_WORKSPACE}/src
$ GIT_LFS_SKIP_SMUDGE=1 git clone -b humble-devel https://github.com/Alpaca-zip/ultralytics_ros.git 
$ rosdep install -r -y -i --from-paths .
$ python3 -m pip install -r ultralytics_ros/requirements.txt 
$ cd ~/{ROS2_WORKSPACE} && $ colcon build
$ source ~/.bashrc
```

**Note**: make sure **Git Large File Storage (LFS)** is installed already as it is required by **ultralytics_ros **. 

4. Additional information:

It is common to have this error notice when building the workspace during Step 3:

```bash
CMake Error at CmakeLists.txt:16 (find packages)
```

In this case, please try the following methods in order:
   
   (1) Make sure that you have this package installed:
      
   ```bash     
   sudo apt install ros-${ROS_DISTRO}-ament-cmake-clang-format
   ```
   
   ```bash
   sudo apt install ros-humble-ament-cmake-clang-format
   ```
      
   (2) Authorise permissions to edit the workspace directory. Try giving read/write permission of the workspace to all users using:
      
   ```bash
   cd ~/{ROS2_WORKSPACE}
   sudo chmod 777 -R .
   ```
      
   (3) Then build using colcon build in the current directory.
      
   ```bash
   cd ~/{ROS2_WORKSPACE} && $ colcon build
   source ~/.bashrc
   ```

ADD THE SENSOR DRIVER INFORMATION - TO BE COMPLETED

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

