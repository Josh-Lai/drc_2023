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
        points_topic = '/camera/depth/color/points'

        self.subscription = self.create_subscription(
            Image,
            points_topic,
            self.detection_callback,
            100)

        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Point, '/PUT_THE_BLOODY_NAME_NODE_HERE', 10)
        self.blue_limits = [np.array([90, 100, 100]), np.array([120, 255, 255])]
        self.yellow_limits = [np.array([20, 100, 100]), np.array([40, 255, 255])]


    def detection_callback(self, msg):
        self.i+=1
        array = pointcloud2_to_xyz_array(msg)
        print(array)
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

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

DUMMY_FIELD_PREFIX = '__'

type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}

def fields_to_dtype(fields, point_step):
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list


def dtype_to_fields(dtype):
    fields = []
    for field_name in dtype.names:
        np_field_type, field_offset = dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name
        if np_field_type.subdtype:
            item_dtype, shape = np_field_type.subdtype
            pf.count = np.prod(shape)
            np_field_type = item_dtype
        else:
            pf.count = 1

        pf.datatype = nptype_to_pftype[np_field_type]
        pf.offset = field_offset
        fields.append(pf)
    return fields


def pointcloud2_to_array(cloud_msg, squeeze=True):
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)
    cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

    if squeeze and cloud_msg.height == 1:
        return np.reshape(cloud_arr, (cloud_msg.width,))
    else:
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))


def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    cloud_arr = np.atleast_2d(cloud_arr)
    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    cloud_msg.fields = dtype_to_fields(cloud_arr.dtype)
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step * cloud_arr.shape[1]
    cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.data = cloud_arr.tostring()
    return cloud_msg


def merge_rgb_fields(cloud_arr):
    r = np.asarray(cloud_arr['r'], dtype=np.uint32)
    g = np.asarray(cloud_arr['g'], dtype=np.uint32)
    b = np.asarray(cloud_arr['b'], dtype=np.uint32)
    rgb_arr = np.array((r << 16) | (g << 8) | (b << 0), dtype=np.uint32)
    rgb_arr.dtype = np.float32

    new_dtype = []
    for field_name in cloud_arr.dtype.names:
        field_type, field_offset = cloud_arr.dtype.fields[field_name]
        if field_name not in ('r', 'g', 'b'):
            new_dtype.append((field_name, field_type))
    new_dtype.append(('rgb', np.float32))
    new_cloud_arr = np.zeros(cloud_arr.shape, new_dtype)

    for field_name in new_cloud_arr.dtype.names:
        if field_name == 'rgb':
            new_cloud_arr[field_name] = rgb_arr
        else:
            new_cloud_arr[field_name] = cloud_arr[field_name]

    return new_cloud_arr


def split_rgb_field(cloud_arr):
    rgb_arr = cloud_arr['rgb'].copy()
    rgb_arr.dtype = np.uint32
    r = np.asarray((rgb_arr >> 16) & 255, dtype=np.uint8)
    g = np.asarray((rgb_arr >> 8) & 255, dtype=np.uint8)
    b = np.asarray(rgb_arr & 255, dtype=np.uint8)

    new_dtype = []
    for field_name in cloud_arr.dtype.names:
        field_type, field_offset = cloud_arr.dtype.fields[field_name]
        if not field_name == 'rgb':
            new_dtype.append((field_name, field_type))
    new_dtype.append(('r', np.uint8))
    new_dtype.append(('g', np.uint8))
    new_dtype.append(('b', np.uint8))
    new_cloud_arr = np.zeros(cloud_arr.shape, new_dtype)

    for field_name in new_cloud_arr.dtype.names:
        if field_name == 'r':
            new_cloud_arr[field_name] = r
        elif field_name == 'g':
            new_cloud_arr[field_name] = g
        elif field_name == 'b':
            new_cloud_arr[field_name] = b
        else:
            new_cloud_arr[field_name] = cloud_arr[field_name]

    return new_cloud_arr


def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float):
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]

    points = np.zeros(cloud_array.shape + (3,), dtype=dtype)
    points[..., 0] = cloud_array['x']
    points[..., 1] = cloud_array['y']
    points[..., 2] = cloud_array['z']

    return points


def pointcloud2_to_xyz_array(cloud_msg, remove_nans=True):
    return get_xyz_points(pointcloud2_to_array(cloud_msg), remove_nans=remove_nans)
