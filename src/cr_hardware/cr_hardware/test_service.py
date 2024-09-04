#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cr_interfaces.srv import GetNextWaypoint  # Replace with your package name

class GetNextWaypointServer(Node):

    def __init__(self):
        super().__init__('get_next_waypoint_server')
        self.srv = self.create_service(GetNextWaypoint, 'get_next_waypoint', self.get_next_waypoint_callback)

    def get_next_waypoint_callback(self, request, response):
        self.get_logger().info(f'Received request with threshold: {request.threshold}')

        # Example logic to determine column and row
        if request.threshold > 10.0:
            response.column = 1
            response.row = 2
        else:
            response.column = 0
            response.row = 1

        self.get_logger().info(f'Responding with column: {response.column}, row: {response.row}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GetNextWaypointServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
