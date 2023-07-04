import cv2 
import os
import numpy as np

lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([40, 255, 255])
lower_blue = np.array([90, 100, 100])
upper_blue = np.array([120, 255, 255])

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

def get_images(directory, filetype):
    image_list = []
    
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(filetype):
                image_path = os.path.join(root, file)
                image_list.append(image_path)
    
    return image_list

def find_single_lane(image, lower_thresh, upper_thresh):
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

def contour_to_string(contour, class_index, xmax, ymax):
    # Convert contour to the specified format
    contour_string = f"{class_index}"
    for point in contour:
        x, y = point[0]
        contour_string += f" {x/xmax} {y/ymax}"
    return contour_string

def check_segmentation(image):
	
	while (True):
		copy = image.copy()
		blue_lane = find_single_lane(image, lower_blue, upper_blue)
		yellow_lane = find_single_lane(image, lower_yellow, upper_yellow)

		x_blue,y_blue,w_blue,h_blue = cv2.boundingRect(blue_lane)
		cv2.rectangle(copy, (x_blue,y_blue), (x_blue+w_blue,y_blue+h_blue), (0,255,0), 2)
		x_yellow,y_yellow,w_yellow,h_yellow = cv2.boundingRect(yellow_lane)
		cv2.rectangle(copy, (x_yellow,y_yellow), (x_yellow+w_yellow,y_yellow+h_yellow), (0,255,0), 2)

		cv2.drawContours(copy, yellow_lane, -1, (0, 255, 255), 2)
		cv2.drawContours(copy, blue_lane, -1, (255, 0, 0), 2)
		
		cv2.imshow("Detected Lanes", copy)
		key = cv2.waitKey(1)
		if key == ord("q"):
			break
	
	yellow_data = contour_to_string(yellow_lane, 0, image.shape[1], image.shape[0])
	blue_data = contour_to_string(blue_lane, 1, image.shape[1], image.shape[1])

	return f"{yellow_data}\n{blue_data}"
	

	
	
def main():

	global lower_yellow, upper_yellow, lower_blue, upper_blue
	# Create a window to display the trackbars
	cv2.namedWindow('Trackbars')
	cv2.createTrackbar('Y Lower H', 'Trackbars', lower_yellow[0], 255, update_values)
	cv2.createTrackbar('Y Lower S', 'Trackbars', lower_yellow[1], 255, update_values)
	cv2.createTrackbar('Y Lower V', 'Trackbars', lower_yellow[2], 255, update_values)
	cv2.createTrackbar('Y Upper H', 'Trackbars', upper_yellow[0], 255, update_values)
	cv2.createTrackbar('Y Upper S', 'Trackbars', upper_yellow[1], 255, update_values)
	cv2.createTrackbar('Y Upper V', 'Trackbars', upper_yellow[2], 255, update_values)

	cv2.createTrackbar('B Lower H', 'Trackbars', lower_blue[0], 255, update_values)
	cv2.createTrackbar('B Lower S', 'Trackbars', lower_blue[1], 255, update_values)
	cv2.createTrackbar('B Lower V', 'Trackbars', lower_blue[2], 255, update_values)
	cv2.createTrackbar('B Upper H', 'Trackbars', upper_blue[0], 255, update_values)
	cv2.createTrackbar('B Upper S', 'Trackbars', upper_blue[1], 255, update_values)
	cv2.createTrackbar('B Upper V', 'Trackbars', upper_blue[2], 255, update_values)

	update_values()

	textfile_path = "images/textfiles/"		

	# Load images from folder	
	image_path_list = get_images("images", "jpg")
	
	# For each image find the lane (output the [contour] points of the lane)
	for image_path in image_path_list:
		image = cv2.imread(image_path)	
		
		# Check if lanes are correct		
		data = check_segmentation(image)
		base_filename = image_path.split("/")[-1].split(".")[0]

		# Create the new file path with the desired extension
		text_path  = textfile_path + base_filename + ".txt"
		with open(text_path, 'w') as file:
			file.write(data)
	return


if __name__ == "__main__":
	main()


