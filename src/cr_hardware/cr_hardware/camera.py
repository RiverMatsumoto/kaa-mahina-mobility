import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DualCameraNode(Node):
    def __init__(self):
        super().__init__('dual_camera_node')

        # Initialize video capture for two cameras
        self.cap1 = cv2.VideoCapture(0)  # /dev/video0
        self.cap2 = cv2.VideoCapture(2)  # /dev/video1

        # Check if cameras opened successfully
        if not self.cap1.isOpened():
            self.get_logger().error('Failed to open video device 0')
            return
        if not self.cap2.isOpened():
            self.get_logger().error('Failed to open video device 1')
            return

        self.publisher1 = self.create_publisher(Image, 'camera1/image_raw', 10)
        self.publisher2 = self.create_publisher(Image, 'camera2/image_raw', 10)
        self.bridge = CvBridge()

        # Set timer to periodically capture and publish frames
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret1, frame1 = self.cap1.read()
        ret2, frame2 = self.cap2.read()

        if ret1:
            msg1 = self.bridge.cv2_to_imgmsg(frame1, encoding='bgr8')
            self.publisher1.publish(msg1)
        else:
            self.get_logger().error('Failed to read frame from video device 0')

        if ret2:
            msg2 = self.bridge.cv2_to_imgmsg(frame2, encoding='bgr8')
            self.publisher2.publish(msg2)
        else:
            self.get_logger().error('Failed to read frame from video device 1')

    def destroy_node(self):
        self.cap1.release()
        self.cap2.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
