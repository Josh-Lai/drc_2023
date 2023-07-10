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

lower_yellow = np.array([0, 0, 0])
upper_yellow = np.array([0, 0, 0])
lower_blue = np.array([0, 0, 0])
upper_blue = np.array([0, 0, 0])

def update_values(*args):
    global lower_yellow, upper_yellow, lower_blue, upper_blue
    lower_yellow = np.array([cv2.getTrackbarPos('Y Lower H', 'Trackbars'),
                             cv2.getTrackbarPos('Y Lower S', 'Trackbars'),
                             cv2.getTrackbarPos('Y Lower V', 'Trackbars')])
    upper_yellow = np.array([cv2.getTrackbarPos('Y Upper H', 'Trackbars'),
                             cv2.getTrackbarPos('Y Upper S', 'Trackbars'),
                             cv2.getTrackbarPos('Y Upper V', 'Trackbars')])
    lower_blue = np.array([cv2.getTrackbarPos('B Lower H', 'Trackbars'),
                           cv2.getTrackbarPos('B Lower S', 'Trackbars'),
                           cv2.getTrackbarPos('B Lower V', 'Trackbars')])
    upper_blue = np.array([cv2.getTrackbarPos('B Upper H', 'Trackbars'),
                           cv2.getTrackbarPos('B Upper S', 'Trackbars'),
                           cv2.getTrackbarPos('B Upper V', 'Trackbars')])

class Detection(Node):

    def __init__(self):
        # global lower_yellow, upper_yellow, lower_blue, upper_blue
        # Create a window to display the trackbars
        cv2.namedWindow('Trackbars')
        cv2.createTrackbar('Y Lower H', 'Trackbars', 20, 255, update_values)
        cv2.createTrackbar('Y Lower S', 'Trackbars', 0, 255, update_values)
        cv2.createTrackbar('Y Lower V', 'Trackbars', 50, 255, update_values)
        cv2.createTrackbar('Y Upper H', 'Trackbars', 73, 255, update_values)
        cv2.createTrackbar('Y Upper S', 'Trackbars', 255, 255, update_values)
        cv2.createTrackbar('Y Upper V', 'Trackbars', 255, 255, update_values)

        cv2.createTrackbar('B Lower H', 'Trackbars', 100, 255, update_values)
        cv2.createTrackbar('B Lower S', 'Trackbars', 146, 255, update_values)
        cv2.createTrackbar('B Lower V', 'Trackbars', 147, 255, update_values)
        cv2.createTrackbar('B Upper H', 'Trackbars', 107, 255, update_values)
        cv2.createTrackbar('B Upper S', 'Trackbars', 255, 255, update_values)
        cv2.createTrackbar('B Upper V', 'Trackbars', 255, 255, update_values)

        # update_values()


        super().__init__('cv_detection')
        self.i = 0
        image_topic = '/camera/color/image_raw'

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.detection_callback,
            100)
        self.publisher = self.create_publisher(LaneMask, '/lane_bit_mask', 1)

        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.blue_limits = [np.array([90, 100, 100]), np.array([120, 255, 255])]
        self.yellow_limits = [np.array([20, 100, 100]), np.array([40, 255, 255])]


    def detection_callback(self, msg):
        self.i+=1
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR )
        mask = self.get_lane_data(cv_image, [lower_yellow, upper_yellow], [lower_blue, upper_blue])
        msg = LaneMask()

        msg.lane_mask.data = mask.flatten().tolist()

        # this code aint much and its shiet, but its honest work
        height = Int32()
        height.data = mask.shape[0]
        width = Int32()
        width.data = mask.shape[1]

        msg.height = height
        msg.width = width

        self.publisher.publish(msg)
        self.get_logger().info('%d Mask Published' % self.i)
        cv2.waitKey(1)  

    # Returns the detected yellow and blue lanes within the image
    # yellow_limits/blue_limits is just a tuple of lower_hsv_range,upper_hsv_range
    def get_lane_data(self, image, yellow_limits, blue_limits):
        yellow_lanes =  self._find_single_lane(image, yellow_limits[0], yellow_limits[1], "yellow")
        blue_lanes = self._find_single_lane(image, blue_limits[0], blue_limits[1], "blue")
        
        cv2.imshow("yellow", yellow_lanes)
        cv2.imshow("blue", blue_lanes)
        cv2.imshow("orig", image)
        combined_mask = yellow_lanes | blue_lanes
        image = cv2.bitwise_and(image, image, mask=combined_mask)

        return combined_mask

    def convert_contour_to_points(self, contour):
        point_msgs = []

        for point in contour:
            point_msg = Point()
            point_msg.x = point[0][0]
            point_msg.y = point[0][1]
            point_msg.z = 0.0  # Set the z-coordinate if needed
            point_msgs.append(point_msg)

        return point_msgs

    def _find_single_lane(self, image, lower_thresh, upper_thresh, name):
        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold the image to obtain a mask of the lane
        mask = cv2.inRange(hsv_image, lower_thresh, upper_thresh)

        return mask
    

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = Detection()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()