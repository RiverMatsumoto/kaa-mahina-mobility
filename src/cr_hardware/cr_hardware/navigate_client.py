#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cr_interfaces.action import Navigate

class NavigateActionClient(Node):
    def __init__(self):
        super().__init__('navigate_action_client')
        self._action_client = ActionClient(self, Navigate, 'navigate')

    def send_goal(self, utm_zone, utm_easting, utm_northing):
        goal_msg = Navigate.Goal()
        goal_msg.utm_zone = utm_zone
        goal_msg.utm_easting = utm_easting
        goal_msg.utm_northing = utm_northing

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters')

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Navigation succeeded!')
        else:
            self.get_logger().info('Navigation failed.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    action_client = NavigateActionClient()

    # Send UTM goal (update these values)
    # (04Q, 622769.35, 2355610.87)
    utm_zone = '04Q'
    utm_easting_goal = 622771.0  # Example goal
    utm_northing_goal = 2355614.0  # Example goal

    action_client.send_goal(utm_zone, utm_easting_goal, utm_northing_goal)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
