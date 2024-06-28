
import rclpy
import csv
from rclpy.node import Node
from example_interfaces.srv import SetBool
from example_interfaces.msg import String
import subprocess

"""
    bag files need:
        trial type 
        date and time
        notes.txt to go along with it
    
    
"""

class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')
        self.stop_srv = self.create_service(SetBool, 'stop_recording', self.stop_recording_callback)
        self.start_srv = self.create_service(SetBool, 'start_recording', self.start_recording_callback)
        self.process = None

    def start_recording(self, request, response):
        output_bag = request.output_bag
        topics = request.topics
        command = ['ros2', 'bag', 'record', '-o', output_bag] + topics
        self.process = subprocess.Popen(command)
        self.get_logger().info('Started recording')

    def stop_recording_callback(self, request, response):
        if self.process:
            self.process.terminate()
            self.process.wait()
            self.process = None
            self.get_logger().info('Stopped recording')
            response.success = True
        else:
            self.get_logger().warn('No recording process found')
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    bag_recorder = BagRecorder()
    
    topics_to_record = [
        "/test"
    ]

    bag_recorder.start_recording(topics_to_record)
    rclpy.spin(bag_recorder)

    bag_recorder.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()