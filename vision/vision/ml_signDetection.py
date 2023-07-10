from ultralytics import YOLO
import cv2
import math 
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from cv_bridge import CvBridge
import cv2
import numpy as np

class SignDetection(Node): 

    def __init__(self):
        super().__init__("ml_sign_detection") 
        self.model = YOLO("/home/shanker/Downloads/train2/weights/best.pt")

        self.image_subscription = self.create_subscription( 
            Image,
            'camera/color/image_raw',
            self.image_callback, 
            1) 
        self.bridge = CvBridge() 
        self.publisher 
        self.direction_publisher = self.create_publisher(String, '/sign_direction') 
        self.class_names = ['left', 'right']

    def image_callback(self, msg): 
        img = self.bridge.imgmsg_to_cv2(msg)
        img = cv2.resize(img, (640,640))
        results = self.model(img, stream=True)
        
        # coordinates
        for r in results:
            boxes = r.boxes
            conf_scores = []
            class_scores = []

            for box in boxes:
                # bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

                # put box in cam

                # confidence
                confidence = math.ceil((box.conf[0]*100))/100
                conf_scores.append(confidence)
                print("Confidence --->",confidence)

                # class name
                cls = int(box.cls[0])
                class_scores.append(self.class_names[cls])
                print("Class name -->", self.class_names[cls])

                # object details
                org = [x1, y1]
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (255, 0, 0)
                thickness = 2
                

            if len(conf_scores)>0:
                if np.max(conf_scores)> 0.8:
                    msg = String() 
                    msg.data = self.class_names[np.argmax(conf_scores)]
                    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
                    cv2.putText(img, f"{msg.data} {np.max(conf_scores)}", org, font, fontScale, color, thickness)
                    self.direction_publisher.publish(msg)

            cv2.imshow('Webcam', img)
            if cv2.waitKey(1) == ord('q'):
                break

def main(args=None): 
    rclpy.init(args=args) 
    sign_detection = SignDetection() 
    rclpy.spin(sign_detection) 
    cv2.destroyAllWindows()
    sign_detection.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 
