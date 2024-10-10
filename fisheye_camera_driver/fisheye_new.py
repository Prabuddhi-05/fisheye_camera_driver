#!/usr/bin/env python

import subprocess  # Runs the shell commands 
import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import numpy as np
import pyudev  # For working with device nodes

class FisheyeCameraNode(Node): 
    def __init__(self):
        super().__init__('fisheye_camera_node')
        # Declare the serial_no parameter with a default value
        self.serial_no = self.declare_parameter('serial_no', 'SN00000').value  # Default value can be set
        self.video_device = None
        self.ffmpeg_process = None  # Initialize ffmpeg_process as None
        self.find_video_number(self.serial_no)  # Finds the video device number based on the serial number
        
        if self.video_device is None:
            self.get_logger().error(f"Error: H264 format is not found on the device with serial number {self.serial_no}.")
            return

        self.get_logger().info(f"Found camera device at /dev/video{self.video_device} with serial number {self.serial_no}")
        self.topic_name = f"/fisheye_image_{self.serial_no}"  # Generates a topic name based on the serial number
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                          history=HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.publisher = self.create_publisher(Image, self.topic_name, qos_policy)  # Creates a publisher for the Image messages on the topic 
        #self.publisher = self.create_publisher(Image, self.topic_name, 10)  # Creates a publisher for the Image messages on the topic 

        # Updated ffmpeg command to capture at 640x480 resolution
        command = f"ffmpeg -fflags nobuffer -framerate 30 -video_size 640x360 -i /dev/video{self.video_device} -pix_fmt bgr24 -f rawvideo -"  
        
        try:
            self.ffmpeg_process = subprocess.Popen(command.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=10**8)  # Starts the ffmpeg process
        except Exception as e:
            self.get_logger().error(f"Error starting ffmpeg process: {e}")
            return

        self.bridge = CvBridge() 
        # self.timer = self.create_timer(0.05, self.publish_video)

    def find_video_number(self, serial_number):  # Finds the video device number based on the serial number
        max_video_number = 50  
        for video_number in range(max_video_number):  # Iterates through possible video device numbers
            try:
                if self.getSerialNumber(video_number) == serial_number:  # Checks if the serial number matches
                    command = f"v4l2-ctl --list-formats-ext -d {video_number}"  # Lists formats for each video feed
                    result = subprocess.run(command, shell=True, capture_output=True, text=True)  # Captures the output of the shell command
                    if "H264" in result.stdout:  # Checks for H264 formats in the output and returns the video number if the appropriate format is found
                        self.video_device = video_number  
                        return
            except (FileNotFoundError, pyudev._errors.DeviceNotFoundByFileError): 
                continue
            except Exception as e:
                self.get_logger().error(f"Error finding video device: {e}")

    def getSerialNumber(self, device=0):  # Gets the serial number of a video device
        context = pyudev.Context()
        device_file = f"/dev/video{device}"
        try:
            device = pyudev.Devices.from_device_file(context, device_file)
            info = {item[0]: item[1] for item in device.items()}
            return info.get("ID_SERIAL_SHORT")
        except (FileNotFoundError, pyudev._errors.DeviceNotFoundByFileError):
            return None

    def publish_video(self):  # Publishes the video frames
        if self.ffmpeg_process is None:  # Check if ffmpeg_process was successfully created
            self.get_logger().error("ffmpeg_process is not initialized. Exiting publish_video.")
            return
            
        while True:  # Continuously read and publish frames
            try:
                print("================ff===========")
                raw_frame = self.ffmpeg_process.stdout.read(640 * 360 * 3)  # Reads raw video frame data from the ffmpeg process (640x480 resolution, 3 channels for BGR)
                print("================ff===========")
                if len(raw_frame) != 640 * 360 * 3:
                    print(len(raw_frame))
                    continue  # Skip incomplete frames
                print(len(raw_frame))
                frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((360, 640, 3))  # Converts the raw frame data to a numpy array (h,w,channels)
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8") 
                self.publisher.publish(ros_image)
            except Exception as e:
                print("Error {}".format(e))
                pass 

def main(args=None):
    rclpy.init(args=args)
    
    video_publisher = FisheyeCameraNode()

    try:
        # Continuously process and publish video frames
        video_publisher.publish_video()
    except KeyboardInterrupt:
        video_publisher.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        # Cleanup ROS2 nodes
        video_publisher.destroy_node()
    # rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

