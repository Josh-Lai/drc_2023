import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Detection(Node):

    def __init__(self):
        super().__init__('cv_detection')
        self.subscription = self.create_subscription(
            Image,
            'test',
            self.detection_callback,
            10)
        self.subscription  # prevent unused variable warning
        bridge = CvBridge()

    def detection_callback(self, msg):
        cv_image = bridge.imgmsg_to_cv2(msg)
        cv2.imshow("testing", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Detection()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()