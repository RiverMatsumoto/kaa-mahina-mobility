#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from cr_interfaces.action import Navigate
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import math
from pyproj import Proj

class PIDNavigationNode(Node):
    def __init__(self):
        super().__init__('pid_navigation_node')

        # Action Server
        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate',
            self.execute_callback)

        # Odometry subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odom_callback,
            10)

        # Velocity publisher
        self.publisher_ = self.create_publisher(TwistStamped, '/differential_drive_controller/cmd_vel', 10)

        # Current pose variables (initialized later)
        self.x = None
        self.y = None
        self.theta = None

        # PID parameters
        self.kp_linear = 0.5
        self.kp_angular = 2.0

        # Tolerance
        self.distance_tolerance = 0.3  # 30 centimeters

        # UTM projection (to be initialized)
        self.proj_utm = None

        # Initial UTM position (your current position)
        # (04Q, 622769.35, 2355610.87)
        self.initial_utm_zone = '04Q'
        self.initial_utm_easting = 622769.35
        self.initial_utm_northing = 2355610.87
        self.initial_theta = 0.0  # Facing north

        # Initialize projection
        zone_number = int(self.initial_utm_zone[:-1])
        hemisphere = self.initial_utm_zone[-1]
        self.proj_utm = Proj(proj='utm', zone=zone_number, south=hemisphere < 'N')

    def odom_callback(self, msg):
        # Update the current position and orientation from odometry
        self.x = msg.pose.pose.position.x + self.initial_utm_easting
        self.y = msg.pose.pose.position.y + self.initial_utm_northing
        self.theta = self.get_yaw_from_quaternion(msg.pose.pose.orientation) + self.initial_theta

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received navigation goal')

        # Wait until we have received the initial pose
        while self.x is None or self.y is None:
            rclpy.spin_once(self)

        # Get the goal parameters
        utm_zone = goal_handle.request.utm_zone
        utm_easting_goal = goal_handle.request.utm_easting
        utm_northing_goal = goal_handle.request.utm_northing

        self.get_logger().info(f'Navigating to UTM Zone: {utm_zone}, Easting: {utm_easting_goal}, Northing: {utm_northing_goal}')

        # Ensure the UTM zones match
        if utm_zone != self.initial_utm_zone:
            self.get_logger().error('UTM zone mismatch between current position and goal.')
            goal_handle.abort()
            result = Navigate.Result()
            result.success = False
            return result

        # Control loop
        result = Navigate.Result()
        feedback_msg = Navigate.Feedback()
        rate = self.create_rate(10)  # 10 Hz

        while rclpy.ok():
            # Calculate deltas
            delta_easting = utm_easting_goal - self.x
            delta_northing = utm_northing_goal - self.y

            # Calculate errors
            distance = math.hypot(delta_easting, delta_northing)
            angle_to_goal = math.atan2(delta_northing, delta_easting)
            angle_error = self.normalize_angle(angle_to_goal - self.theta)

            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Navigation goal canceled')
                goal_handle.canceled()
                return Navigate.Result()

            # Update feedback
            feedback_msg.distance_remaining = distance
            goal_handle.publish_feedback(feedback_msg)

            # Control commands
            twist = TwistStamped()
            twist.twist.linear.x = self.kp_linear * distance
            twist.twist.angular.z = self.kp_angular * angle_error

            # Stop conditions
            if distance < self.distance_tolerance:
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = 0.0
                self.publisher_.publish(twist)
                self.get_logger().info('Goal reached within tolerance')
                result.success = True
                goal_handle.succeed()
                return result

            # Publish commands
            self.publisher_.publish(twist)

            # Sleep
            rate.sleep()

        # If loop exits unexpectedly
        goal_handle.abort()
        result.success = False
        return result

    def get_yaw_from_quaternion(self, orientation_q):
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y ** 2 + orientation_q.z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)

    pid_navigation_node = PIDNavigationNode()

    rclpy.spin(pid_navigation_node)

    pid_navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
