#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cr_interfaces.srv import GetNextWaypoint
from std_srvs.srv import SetBool

class MockServiceServer(Node):

    def __init__(self):
        super().__init__('mock_service_server')
        self.srv = self.create_service(SetBool, 'lower_probe', self.get_next_waypoint_callback)
        self.get_logger().info('Mock Service Server ready to handle requests.')

    def get_next_waypoint_callback(self, request, response):
        self.get_logger().info(f'Received request: {request}')
        # Provide a canned response
        response = SetBool.Response()
        self.get_logger().info(f'')
        return response

def main(args=None):
    rclpy.init(args=args)
    mock_service_server = MockServiceServer()
    rclpy.spin(mock_service_server)
    mock_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
