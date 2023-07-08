#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Point
import numpy as np
from cv_msgs.msg import LanePixels

class Detection(Node):

    def __init__(self):
        super().__init__('cv_detection')
        self.i = 0
        image_topic = '/camera/color/image_raw'
        # self.declare_parameter('image_topic', '/camera/color/image_raw')
        # image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        # print(image_topic)
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.detection_callback,
            100)

        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Point, '/PUT_THE_BLOODY_NAME_NODE_HERE', 10)
        self.blue_limits = [np.array([90, 100, 100]), np.array([120, 255, 255])]
        self.yellow_limits = [np.array([20, 100, 100]), np.array([40, 255, 255])]


    def detection_callback(self, msg):
        self.i+=1
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        self.get_logger().info('%d Images Received' % self.i)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR )
        self.publisher.publish(self.get_lane_data(cv_image, self.yellow_limits, self.blue_limits))
        cv2.imshow("image_raw", cv_image)
        cv2.waitKey(1)

    # Returns the detected yellow and blue lanes within the image
    # yellow_limits/blue_limits is just a tuple of lower_hsv_range,upper_hsv_range
    def get_lane_data(self, image, yellow_limits, blue_limits):
        yellow_lanes =  self._find_single_lane(image, yellow_limits[0], yellow_limits[1])
        blue_lanes = self._find_single_lane(image, blue_limits[0], blue_limits[1])
        
        # Float XX multi_array
        lane_data = LanePixels()
        lane_data.yellow_lane_pixels = self.convert_contour_to_points(yellow_lanes)
        lane_data.blue_lane_pixels = self.convert_contour_to_points(blue_lanes)

        return lane_data

    def convert_contour_to_points(self, contour):
        point_msgs = []

        for point in contour:
            point_msg = Point()
            point_msg.x = point[0][0]
            point_msg.y = point[0][1]
            point_msg.z = 0.0  # Set the z-coordinate if needed
            point_msgs.append(point_msg)

        return point_msgs

    def _find_single_lane(self, image, lower_thresh, upper_thresh):
        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold the image to obtain a mask of the lane
        mask = cv2.inRange(hsv_image, lower_thresh, upper_thresh)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Find the contour with the largest area
            return max(contours, key=cv2.contourArea)
        
        return None
    

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = Detection()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
