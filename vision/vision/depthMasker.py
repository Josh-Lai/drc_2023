#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Point
import numpy as np
from cv_msgs.msg import LaneMask
from std_msgs.msg import Int32

RECEIVE_MASK = 1
APPLY_MASK = 2

class DepthMasker(Node):

    def __init__(self):
        global lower_yellow, upper_yellow, lower_blue, upper_blue
        # Create a window to display the trackbars

        super().__init__('cv_detection_2')
        self.i = 0
        self.state = RECEIVE_MASK
        self.depth_subscriber = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            100)
        self.mask = np.array([])

        self.mask_subscriber = self.create_subscription(
            LaneMask,
            '/lane_bit_mask',
            self.mask_callback,
            100)
        
        self.publisher = self.create_publisher(Image, '/depth_masked', 1)
        
        self.depth_subscriber  # prevent unused variable warning
        self.mask_subscriber  # prevent unused variable warning

        self.bridge = CvBridge()


    def depth_callback(self, msg):
        self.i+=1
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        print(cv_image)
        
        if self.state == APPLY_MASK and len(self.mask) != 0:
            print(self.mask.shape)
            print(cv_image.shape)
            masked = cv2.bitwise_and(cv_image, cv_image, mask=self.mask)
            
            msg = self.bridge.cv2_to_imgmsg(masked)
            self.publisher.publish(msg)
            
            self.state = RECEIVE_MASK

    def mask_callback(self, msg):
        self.i+=1
        if self.state == RECEIVE_MASK:
            mask = np.array(msg.lane_mask.data,dtype=np.uint8)
            
            h,w = msg.height.data, msg.width.data
            self.mask=cv2.pyrDown(mask.reshape((h,w)))

            self.state = APPLY_MASK


def main(args=None):
    rclpy.init(args=args)

    depth_publisher = DepthMasker()

    rclpy.spin(depth_publisher)

    # Destroy the node explicitly
    depth_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
