import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.get_logger().info('ImageSubscriber node started, listening to /camera/image_raw')

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Red color mask
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 500:  # Only respond to large enough red objects
                M = cv2.moments(largest)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])

                    width = cv_image.shape[1]

                    if cx < width * 0.4:
                        self.get_logger().info(f"Red object at x={cx} → LEFT")
                        twist.angular.z = 0.3
                    elif cx > width * 0.6:
                        self.get_logger().info(f"Red object at x={cx} → RIGHT")
                        twist.angular.z = -0.3
                    else:
                        self.get_logger().info(f"Red object at x={cx} → CENTER")
                        twist.linear.x = 0.15

                    # Stop if very close (large area)
                    if area > 10000:
                        self.get_logger().info("Close to the red object — stopping.")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
            else:
                self.get_logger().info("Red object too small — ignoring.")
        else:
            self.get_logger().info("No red object detected — stopping.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

