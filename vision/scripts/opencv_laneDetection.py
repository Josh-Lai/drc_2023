import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Detection(Node):

    def __init__(self):
        super().__init__('cv_detection')
        
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.detection_callback,
            10)

        self.subscription  # prevent unused variable warning
        bridge = CvBridge()

    def detection_callback(self, msg):
        print("hello")
        cv_image = bridge.imgmsg_to_cv2(msg)
        # cv2.imshow("testing", cv_image)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = Detection()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()