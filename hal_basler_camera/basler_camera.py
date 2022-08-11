#!/usr/bin/env python3

# Libraries
import threading
import sys
import numpy as np
import cv2 as cv

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Header

from cv_bridge import CvBridge

from pypylon import pylon


# Class definition of the calibration function
class BaslerCameraNode(Node):
    def __init__(self):
        super().__init__("basler_cam_node")
        
        self.declare_parameter("camera_name", "auto")

        self.bridge = CvBridge()
        self.frame = []
        self.acquire_frames = True

        self.declare_parameter("publishers.raw_frame", "/camera/raw_frame")
        self.raw_frame_topic = self.get_parameter("publishers.raw_frame").value

        self.declare_parameter("rotation_angle", "0.0")
        self.rotation_angle = float(self.get_parameter("rotation_angle").value)

        self.declare_parameter("frames.camera_link", "camera_link")
        self.camera_link = self.get_parameter("frames.camera_link").value

        self.declare_parameter("camera_ip", "auto")
        self.camera_ip = self.get_parameter("camera_ip").value
        
        self.frame_pub = self.create_publisher(Image, self.raw_frame_topic, 1)

        # converting to opencv bgr format
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        self.thread1 = threading.Thread(target=self.get_frame, daemon=True)
        self.thread1.start()

        self.get_logger().info("[Basler Camera] Node Ready")


    def open_camera(self):
        # Get the transport layer factory.
        tlFactory = pylon.TlFactory.GetInstance()

        # Get all attached devices and exit application if no device is found.
        devices = tlFactory.EnumerateDevices()
        if len(devices) == 0:
            self.get_logger().info("No camera present. Check Connection.")
            return False

        if self.camera_ip == "auto":
            self.get_logger().info("Using First Camera Available")
            self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            return True

        # Create an array of instant cameras for the found devices and avoid exceeding a maximum number of devices.
        cameras = pylon.InstantCameraArray(min(len(devices), 10))

        found = False
        i_found = 0

        # Create and attach all Pylon Devices.
        for i, cam in enumerate(cameras):
            cam.Attach(tlFactory.CreateDevice(devices[i]))
            # Print the model name of the camera.

            if(cam.GetDeviceInfo().GetIpAddress() == self.camera_ip):
                self.get_logger().info("Using device with IP {0}".format(cam.GetDeviceInfo().GetIpAddress()))
                found = True
                i_found = i

        if found:
            self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[i_found]))

            self.camera.Open()
            self.camera.GevSCPSPacketSize.SetValue(1000)
        
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) 
            
            return True

        else:
            return False

    # This function save the current frame in a class attribute
    def get_frame(self):

        if self.open_camera():

            while self.acquire_frames and self.camera.IsGrabbing():
            
                grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

                if grabResult.GrabSucceeded():
                    # Access the image data
                    try:
                        image = self.converter.Convert(grabResult)
                        self.frame = image.GetArray()
                        self.publish_frame()
                    except:
                        pass

            grabResult.Release()

        else:
            self.get_logger().info("[Basler Camera] No AV Camera Found. Check Connection.")


    def exit(self):
        self.acquire_frames = False
        self.thread1.join()


    def publish_frame(self):
        
        if len(self.frame) == 0:
            self.get_logger().info("[Basler Camera] No Image Returned")
            return

        if self.rotation_angle != 0.0:
            image_center = tuple(np.array(self.frame.shape[1::-1]) / 2)
            rot_mat = cv.getRotationMatrix2D(image_center, self.rotation_angle, 1.0)
            self.frame = cv.warpAffine(self.frame, rot_mat, self.frame.shape[1::-1], flags=cv.INTER_LINEAR)

        grey = cv.cvtColor(self.frame, cv.COLOR_RGB2GRAY)

        self.image_message = self.bridge.cv2_to_imgmsg(grey, encoding="mono8")
        self.image_message.header = Header()
        self.image_message.header.stamp = self.get_clock().now().to_msg()
        self.image_message.header.frame_id = self.camera_link
        self.frame_pub.publish(self.image_message)


##### Main Function #####
def main(args=None):

    rclpy.init(args=args)
    node = BaslerCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[Basler Camera] Node stopped cleanly')
        node.exit()
    except BaseException:
        node.get_logger().info('[Basler Camera] Exception:', file=sys.stderr)
        node.exit()
        raise
    finally:
        rclpy.shutdown() 


if __name__ == "__main__":
    main()
