#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.time import Time, Duration
from cr_bag.srv import StartTrial

class TrialExecutor(Node):
    def __init__(self):
        super().__init__('trial_executor')
        self.publisher = self.create_publisher(TwistStamped, '/differential_drive_controller/cmd_vel', 10)
        self.declare_parameter('publish_frequency', 10)
        self.declare_parameter('duration', 43)
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.duration = self.get_parameter('duration').value
        self.twist_msg = TwistStamped()

        # Create the service to start the trial
        self.srv = self.create_service(StartTrial, 'execute_trial', self.start_trial_callback)
        self.timer = None
        self.get_logger().info("Initialized Trial Executor")

    def start_trial_callback(self, request, response):
        linear_x = request.speed
        angular_z = 0.0  # Assuming straight motion for simplicity
        self.execute_trial(linear_x, angular_z)
        return response  # No response content needed

    def execute_trial(self, linear_x=0, angular_z=0):
        self.twist_msg.twist.linear.x = float(linear_x)
        self.twist_msg.twist.angular.z = float(angular_z)
        self.start_time = self.get_clock().now()
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
            self.publisher.publish(TwistStamped())
            self.get_logger().info("Finished publishing Twist messages. Closing trial")
            if self.timer:
                self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    trial_executor = TrialExecutor()

    # rclpy.spin(trial_executor)
    # trial_executor.destroy_node()
    # rclpy.shutdown()
    try:
        rclpy.spin(trial_executor)
        trial_executor.destroy_node()
        rclpy.shutdown()
    except:
        trial_executor.get_logger().info('Keyboard Interrupt (SIGINT) received, shutting down...')

if __name__ == '__main__':
    main()
