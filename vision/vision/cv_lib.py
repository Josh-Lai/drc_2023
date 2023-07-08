import cv2 
import numpy as np
from geometry_msgs.msg import Point
from cv_msgs.msg import LanePixels

# Returns the detected yellow and blue lanes within the image
# yellow_limits/blue_limits is just a tuple of lower_hsv_range,upper_hsv_range
def get_lane_data(image, yellow_limits, blue_limits):
    yellow_lanes = _find_single_lane(image, yellow_limits[0], yellow_limits[1])
    blue_lanes = _find_single_lane(image, blue_limits[0], blue_limits[1])
    
    # Float XX multi_array
    lane_data = LanePixels()
    lane_data.yellow_lane_pixels = convert_contour_to_points(yellow_lanes)
    lane_data.blue_lane_pixels = convert_contour_to_points(blue_lanes)

    return lane_data

def convert_contour_to_points(contour):
    point_msgs = []

    for point in contour:
        point_msg = Point()
        point_msg.x = point[0][0]
        point_msg.y = point[0][1]
        point_msg.z = 0.0  # Set the z-coordinate if needed

        point_msgs.append(point_msg)

    return point_msgs

def _find_single_lane(image, lower_thresh, upper_thresh):
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
