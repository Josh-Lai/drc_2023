#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

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

    def detection_callback(self, msg):
        self.i+=1
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        self.get_logger().info('%d Images Received' % self.i)
        cv_image = cv2.cvtColor(src, cv2.COLOR_RGB2BGR )
        cv2.imshow("image_raw", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = Detection()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()