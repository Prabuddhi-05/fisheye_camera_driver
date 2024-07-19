#!/usr/bin/env python

import subprocess # Runs the shell commands 
import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyudev # For working with device nodes

class FisheyeCameraNode(Node): 
    def __init__(self):
        super().__init__('fisheye_camera_node')
        self.serial_no = self.declare_parameter('serial_no').value # Gets the value of the serial number parameter from the launch file
        self.video_device = None
        self.find_video_number(self.serial_no) # Finds the video device number based on the serial number
        if self.video_device is None:
            self.get_logger().error(f"Error: H264 format is not found on the device with serial number {self.serial_no}.")
            return

        self.get_logger().info(f"Found camera device at /dev/video{self.video_device} with serial number {self.serial_no}")
        self.topic_name = f"/fisheye_image_{self.serial_no}" # Generates a topic name based on the serial number
        self.publisher = self.create_publisher(Image, self.topic_name, 10) # Creates a publisher for the Image messages on the topic 

        command = f"ffmpeg -framerate 60 -i /dev/video{self.video_device} -pix_fmt bgr24 -f rawvideo -" # Captures video frames at 60 fps from the specified video device and streams raw BGR24 video data 
        self.ffmpeg_process = subprocess.Popen(command.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE) # Starts the ffmpeg process

        self.bridge = CvBridge() 
        self.timer = self.create_timer(1/60, self.publish_video) # Publishes the video frames at a rate of 60 Hz

    def find_video_number(self, serial_number): # Finds the video device number based on the serial number
        max_video_number = 50  
        for video_number in range(max_video_number): # Iterates through possible video device numbers
            try:
                if self.getSerialNumber(video_number) == serial_number: # Checks if the serial number matches
                    command = f"v4l2-ctl --list-formats-ext -d {video_number}" #  Lists formats for each video feed
                    result = subprocess.run(command, shell=True, capture_output=True, text=True) # Captures the output of the shell command
                    if "H264" in result.stdout: # Checks for H264 formats in the output and returns the video number if the appropriate format is found
                        self.video_device = video_number #   
                        return
            except (FileNotFoundError, pyudev._errors.DeviceNotFoundByFileError): 
                continue
            except Exception as e:
                self.get_logger().error(f"Error finding video device: {e}")
                
    # From https://docs.arducam.com/UVC-Camera/Appilcation-Note/OpenCV-Python-GStreamer-on-linux/

    def getSerialNumber(self, device=0): # Gets the serial number of a video device
        context = pyudev.Context()
        device_file = "/dev/video{}".format(device)
        try:
            device = pyudev.Devices.from_device_file(context, device_file)
            info = {item[0]: item[1] for item in device.items()}
            return info.get("ID_SERIAL_SHORT")
        except (FileNotFoundError, pyudev._errors.DeviceNotFoundByFileError):
            return None

    def publish_video(self): # Publishes the video frames
        if self.video_device is None:
            return

        raw_frame = self.ffmpeg_process.stdout.read(1920 * 1080 * 3) # Reads raw video frame data from the ffmpeg process (1920x1080 (Full HD) resolution, 3 channels for BGR)  
        if len(raw_frame) != 1920 * 1080 * 3: 
            return

        frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((1080, 1920, 3)) # Converts the raw frame data to a numpy array (h,w,channels)
        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8") 
        self.publisher.publish(ros_image) 
 
def main(args=None):
    rclpy.init(args=args)
    node = FisheyeCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:  # Cleanup ROS2 nodes
        node.destroy_node()  
        rclpy.shutdown()

if __name__ == '__main__':
    main()
