
import threading
import serial
from time import sleep

class SerialHandler:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=0.5,
                 write_interval=0.1, max_queue_length=20):
        self.serial_port = serial.Serial(port, baudrate)
        self.serial_port.timeout = timeout
        self.write_interval = write_interval
        self.msg_queue = []
        self.max_queue_length = max_queue_length
        self.write_thread = threading.Thread(target=self.threaded_write)
        self.write_thread.daemon = True
        self.write_thread.start()
    
    def write(self, msg: str):
        # discard message if msg_queue is full
        if len(self.msg_queue) < self.max_queue_length:
            self.msg_queue.append(msg)
    
    def threaded_write(self):
        """
        don't call this function, it is an infinitely polling thread waiting for serial data
        """
            
        while True:
            # heartbeat
            if not self.is_arduino_connected():
                self.close()
                exit()
            if len(self.msg_queue) > 0 and self.serial_port.is_open:
                msg = self.msg_queue.pop(0).encode()
                self.serial_port.write(msg)
            sleep(self.write_interval)

    def is_arduino_connected(self):
        devices = serial.tools.list_ports.comports()
        arduino_vendor_id = 0x2341  # Example Vendor ID (Arduino)
        arduino_product_id = 0x0043  # Example Product ID (Arduino Uno)
        for device in devices:
            if device.vid == arduino_vendor_id and device.pid == arduino_product_id:
                return True
        return False

    def write_then_read(self, msg: str) -> str:
        # flush reads
        self.serial_port.reset_input_buffer()
        self.write(msg)
        # wait for response
        result = ' '
        while result != None and result[0] != '$': # $ indicates data to be read
            result = self.serial_port.readline().strip().decode()
            
        return result[1:]
    
    def close(self):
        self.serial_port.close()