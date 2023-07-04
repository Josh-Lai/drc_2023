import cv2
import os

def save_image(image):
    # Create the "images" directory if it doesn't exist
    if not os.path.exists("images"):
        os.makedirs("images")
    
    # Generate a unique filename for the image
    image_count = len(os.listdir("images"))
    image_path = os.path.join("images", f"image_{image_count}.jpg")
    
    # Save the image to the specified path
    cv2.imwrite(image_path, image)
    print(f"Image saved as {image_path}")

def capture_images():
    # Open the camera
    camera = cv2.VideoCapture(0)
    
    while True:
        # Read a frame from the camera
        ret, frame = camera.read()
        
        # Display the frame
        cv2.imshow("Camera", frame)
        
        # Check for key press
        key = cv2.waitKey(1)
        if key == ord("q"):
            break
            
        if key == ord("s"):
            save_image(frame)
    
    # Release the camera and close all windows
    camera.release()
    cv2.destroyAllWindows()

# Start capturing images
capture_images()

