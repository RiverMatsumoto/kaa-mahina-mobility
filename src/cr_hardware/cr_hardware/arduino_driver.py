
import rclpy
from rclpy.node import Node
from .submodules.serial_handler import SerialHandler
import serial.tools.list_ports

class ArduinoDriver(Node):

    def __init__(self):
        super().__init__('arduino_driver')
        self.device_path = None
        self.baudrate = 115200
        self.prev_connection = False
        self.have_connection = False
        self.arduino = None
        self.create_timer(1.0, self.try_connect)
    
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
        self.arduino = SerialHandler(self.device_path, self.baudrate)
        if self.arduino.serial_port.is_open:
            self.get_logger().info('Connected to arduino')

    def find_arduino(self):
        devices = serial.tools.list_ports.comports()
        # arduino nano
        arduino_vendor_id = 0x2341
        arduino_product_id = 0x0043
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