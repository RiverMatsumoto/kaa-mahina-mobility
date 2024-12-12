#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.time import Time, Duration
from cr_bag.srv import StartTrial

from sensor_msgs.msg import CompressedImage, Image
import cv2
from cv_bridge import CvBridge
import os
import time

class TrialExecutor(Node):
    def __init__(self):
        super().__init__('trial_executor')
        self.publisher = self.create_publisher(TwistStamped, '/differential_drive_controller/cmd_vel', 10)
        self.declare_parameter('publish_frequency', 10)
        self.declare_parameter('duration', 43)
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.duration = self.get_parameter('duration').value
        self.twist_msg = TwistStamped()

        # Topics for the two cameras
        self.camera1_topic = '/rpi1/image_raw/compressed'
        self.camera2_topic = '/rpi2/image_raw/compressed'
        # Initialize CvBridge for converting image data types
        self.bridge = CvBridge()
        # record camera data for the wheels
        self.camera1_sub = self.create_subscription(CompressedImage, self.camera1_topic, self.camera1_callback, 30)
        self.camera2_sub = self.create_subscription(CompressedImage, self.camera2_topic, self.camera2_callback, 30)
        self.recording = False

        # Create the service to start the trial
        self.srv = self.create_service(StartTrial, 'execute_trial', self.start_trial_callback)
        self.timer = None
        self.get_logger().info("Initialized Trial Executor")

    def camera1_callback(self, msg):
        # Only record when we are tracking a trial
        # self.get_logger().info(f"{self.recording}")
        if self.recording:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            
            # Write frame to video file
            self.camera1_writer.write(frame)

    def camera2_callback(self, msg):
        # Only record when we are tracking a trial
        if self.recording:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            
            # Write frame to video file
            self.camera2_writer.write(frame)

    def start_trial_callback(self, request, response):
        if not self.recording:
            linear_x = request.speed
            # use radius from request to calculate angular_z
            # angular velocity = linear velocity / radius
            if request.radius <= 0.2 and request.radius >= 0:
                # turn left in place
                angular_z = linear_x
                linear_x = 0
            elif request.radius >= -0.2 and request.radius <= 0:
                # turn right in place
                angular_z = -linear_x
                linear_x = 0
            elif request.radius == 1000:
                # go straight
                angular_z = 0
            else:
                angular_z = linear_x / request.radius
            self.execute_trial(linear_x, angular_z, request.directory)
        return response  # No response content needed

    def execute_trial(self, linear_x=0, angular_z=0, directory=""):
        # Initialize VideoWriters for MP4 files
        self.get_logger().info(f"Video files will be saved to the directory: {directory}")
        self.trial_directory = directory
        self.video1_path = os.path.join(self.trial_directory, "camera_left.mp4")
        self.video2_path = os.path.join(self.trial_directory, "camera_right.mp4")
        self.get_logger().info(f"Video files will be saved to the directory: {self.video1_path}")
        self.camera1_writer = cv2.VideoWriter(self.video1_path, cv2.VideoWriter_fourcc(*'mp4v'), 20, (640, 480))
        self.camera2_writer = cv2.VideoWriter(self.video2_path, cv2.VideoWriter_fourcc(*'mp4v'), 20, (640, 480))

        self.twist_msg.twist.linear.x = float(linear_x)
        self.twist_msg.twist.angular.z = float(angular_z)
        self.start_time = self.get_clock().now()
        self.recording = True
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.trial_drive_cb)

    def trial_drive_cb(self):
        elapsed_time = self.get_clock().now() - self.start_time
        if elapsed_time < Duration(seconds=1):
            self.publisher.publish(TwistStamped())
        elif elapsed_time < Time(seconds=self.duration) - Time(seconds=5):
            self.publisher.publish(self.twist_msg)
        elif elapsed_time < Duration(seconds=self.duration):
            self.publisher.publish(TwistStamped())
        else:
            # Stop the timer after the duration has passed
            # Release the VideoWriters when stopping trial
            self.recording = False
            self.camera1_writer.release()
            self.camera2_writer.release()
            self.publisher.publish(TwistStamped())
            self.get_logger().info("Finished publishing Twist messages. Closing trial")
            if self.timer:
                self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    trial_executor = TrialExecutor()

    rclpy.spin(trial_executor)
    trial_executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
