#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial.tools.list_ports

# serial handler
import threading
import serial

class SerialHandler:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=0.5,
                 read_interval=0.1, max_queue_length=20, serial_data_callback=None):
        self.serial_port = serial.Serial(port, baudrate)
        self.serial_port.timeout = timeout

        # will pass the serial data read to this function
        self.serial_data_callback = serial_data_callback

        # interval to prevent arduino serial from overflowing
        self.read_interval = read_interval

        self.msg_queue = []
        self.max_queue_length = max_queue_length

        self.read_thread = threading.Thread(target=self.threaded_read)
        self.read_thread.daemon = True
        self.read_thread.start()
    
    def write(self, msg: str):
        # discard message if msg_queue is full
        if len(self.msg_queue) < self.max_queue_length:
            self.msg_queue.append(msg)
    
    def threaded_read(self):
        while True:
            if self.serial_port.is_open and self.serial_port.in_waiting > 0:
                try:
                    line = self.serial_port.readline().strip().decode('utf-8')
                    if line:
                        self.serial_data_callback(line)
                except Exception as e:
                    print(f"Error reading serial data or sending data to callback: {e}")
                    break
            threading.Event().wait(self.read_interval)  # Control read rate

    def is_arduino_connected(self):
        devices = serial.tools.list_ports.comports()
        arduino_vendor_id = 0x2341  # Example Vendor ID (Arduino)
        arduino_product_id = 0x0043  # Example Product ID (Arduino Uno)
        for device in devices:
            if device.vid == arduino_vendor_id and device.pid == arduino_product_id:
                return True
        return False
    
    def close(self):
        self.serial_port.close()

# arduino driver using serial handler
class ArduinoDriver(Node):
    def __init__(self):
        super().__init__('arduino_driver')
        self.device_path = None
        self.baudrate = 9600
        self.prev_connection = False
        self.have_connection = False
        self.arduino = None
        self.create_timer(1.0, self.try_connect)
        self.moisture_percent_pub = self.create_publisher(Float32, 'moisture_percent', 10)

    def handle_serial_message(self, message: str):
        try:
            # Convert the incoming string message to a float
            moisture_value = float(message.split(',')[-1])
            
            # Create a Float32 message and publish it
            msg = Float32()
            msg.data = moisture_value
            self.moisture_percent_pub.publish(msg)
        except ValueError:
            self.get_logger().warn(f'Unable to convert message to float: {message}')
    
    def try_connect(self):
        # check that usb is plugged in so we can try to connect
        arduino_path = self.find_arduino()
        if not self.have_connection and arduino_path != None:
            self.get_logger().info(f'Found arduino at port: {self.device_path}')
            self.device_path = arduino_path
            self.connect()
        # check that we went from connection to no connection
        self.have_connection = self.arduino.serial_port.is_open
        if not self.have_connection and self.prev_connection:
            self.get_logger().info('Lost connection arduino, will wait for reconnection')
        self.prev_connection = self.have_connection

    def connect(self):
        self.arduino = SerialHandler(self.device_path, self.baudrate, serial_data_callback=self.handle_serial_message)
        if self.arduino.serial_port.is_open:
            self.get_logger().info('Connected to arduino')

    def find_arduino(self):
        devices = serial.tools.list_ports.comports()
        # arduino nano
        arduino_vendor_id = 0x1a86
        arduino_product_id = 0x7523
        for device in devices:
            if device.vid == arduino_vendor_id and device.pid == arduino_product_id:
                return device.device
        return None

def main(args=None):
    rclpy.init(args=args)

    arduino_driver_node = ArduinoDriver()
    rclpy.spin(arduino_driver_node)

    arduino_driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()