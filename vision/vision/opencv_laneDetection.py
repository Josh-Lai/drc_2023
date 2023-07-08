#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Point
import numpy as np
from cv_msgs.msg import LanePixels
import struct
import ctypes
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Header

class Detection(Node):

    def __init__(self):
        super().__init__('cv_detection')
        self.i = 0
        points_topic = '/camera/depth/color/points'

        self.subscription = self.create_subscription(
            PointCloud2,
            points_topic,
            self.detection_callback,
            1)

        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(PointCloud2, '/PUT_THE_BLOODY_NAME_NODE_HERE', 1)
        self.blue_limits = [np.array([35, 6, 112]), np.array([155, 255, 255])]
        self.yellow_limits = [np.array([20, 100, 100]), np.array([40, 255, 255])]
        
    def get_cloud(self, pc2_msg):
        pcl_data = []
        rgb_data = []
        gen = pc2.read_points(pc2_msg, skip_nans=True, field_names=("x", "y", "z", "rgb"))
        for data in gen:            
            test = data[3]
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            pack = ctypes.c_uint32(i).value
            r = int((pack & 0x00FF0000)>> 16)
            g = int((pack & 0x0000FF00)>> 8)
            b = int((pack & 0x000000FF))
            pcl_data.append([data[0], 0.0, data[2], data[3]]) # changed it to zero to align with 2D slam
            rgb_data.append([b,g,r])
        return np.array(pcl_data), np.array(rgb_data, dtype=np.uint8)
    
    
    def detection_callback(self, msg):
        self.i+=1
        self.get_logger().info('%d Received Point Cloud' % self.i)
        pcl_data, rgb_data = self.get_cloud(msg)
        rgb_data = np.reshape(rgb_data, (1, len(rgb_data), 3))
        filtered_pcl = self.filter_pcl(pcl_data, rgb_data)
        print((len(pcl_data) - len(filtered_pcl))/len(pcl_data))
        print(filtered_pcl)
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    # PointField('rgb', 12, PointField.UINT32, 1),
                    PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1)
                    ]

        header = Header()
        header.frame_id = "camera_depth_optical_frame"
        pcl2_msg = pc2.create_cloud(header, fields, filtered_pcl)

        self.publisher.publish(pcl2_msg)

        
    def filter_pcl(self, pcl_data, rgb_data):
        blue_mask = cv2.inRange(rgb_data, self.blue_limits[0], self.blue_limits[1])
        yellow_mask = cv2.inRange(rgb_data, self.yellow_limits[0], self.yellow_limits[1])
        
        combined_mask = (yellow_mask | blue_mask).astype(np.bool_)[0]

        print(combined_mask)
        print(pcl_data)
        masked_pcl = pcl_data[np.array(combined_mask)]
        print(masked_pcl)
        print(np.any(combined_mask))
        return masked_pcl.tolist()


        


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = Detection()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
