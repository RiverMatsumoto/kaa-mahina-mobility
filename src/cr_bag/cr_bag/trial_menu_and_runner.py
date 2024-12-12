#!/usr/bin/env python3
import sys
import signal
import time
import asyncio
from datetime import datetime
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLineEdit, QPushButton, QCheckBox, QLabel, QFileDialog, QComboBox, QHBoxLayout
from PyQt5.QtCore import QRunnable, pyqtSlot, QThreadPool, QCoreApplication, QTimer

# ros2 imports
import rclpy.service
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from cr_bag.srv import StartBag, StopBag, StartTrial

# sleep is for waiting for the ros2 bag record script to actually subscribe to topics and collect data
from time import sleep

class AsyncTask(QRunnable):
    def __init__(self, coro):
        super().__init__()
        self.coro = coro

    @pyqtSlot()
    def run(self):
        asyncio.run(self.coro)

class BagBuilder(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROS Bag Trial Builder')
        self.setGeometry(100, 100, 600, 200)
        
        # ros client setup
        rclpy.init()
        self.node_handle = rclpy.create_node('bag_builder')  # Create a temporary node
        self.start_bag_client = self.node_handle.create_client(StartBag, 'start_recording')  # Replace 'add_two_ints' with your service name
        self.stop_bag_client = self.node_handle.create_client(StopBag, 'stop_recording')  # Replace 'add_two_ints' with your service name
        self.start_trial_client = self.node_handle.create_client(StartTrial, 'execute_trial')

        layout = QVBoxLayout()

        slope_angle_layout = QHBoxLayout()
        self.slope_angle_label = QLabel('Slope angle: ')
        self.slope_angle_combo = QComboBox(self)
        self.slope_angle_combo.addItems(['0', '5', '10', '15', '20', '25', '30'])
        slope_angle_layout.addWidget(self.slope_angle_label)
        slope_angle_layout.addWidget(self.slope_angle_combo)
        layout.addLayout(slope_angle_layout)

        speed_layout = QHBoxLayout()
        self.speed_label = QLabel('Speed (cm/s): ')
        self.speed_combo = QComboBox(self)
        self.speed_combo.addItems(['1', '2', '3', '4', '5', 
                                        '6', '7', '8', '9', '10', 
                                        '11', '12', '13', '14'])
        speed_layout.addWidget(self.speed_label)
        speed_layout.addWidget(self.speed_combo)
        layout.addLayout(speed_layout)

        radius_layout = QHBoxLayout()
        self.radius_label = QLabel('Radius (cm): ')
        self.radius_combo = QComboBox(self)
        radiuses = list(range(0, 180, 20))
        radiuses.append('infinity')
        radiuses = map(str, radiuses)
        self.radius_combo.addItems(radiuses)
        radius_layout.addWidget(self.radius_label)
        radius_layout.addWidget(self.radius_combo)
        layout.addLayout(radius_layout)

        turn_dir_layout = QHBoxLayout()
        self.turn_dir_label = QLabel('Turn Direction: ')
        self.turn_dir_combo = QComboBox(self)
        turn_dir = ['left', 'right']
        self.turn_dir_combo.addItems(turn_dir)
        turn_dir_layout.addWidget(self.turn_dir_label)
        turn_dir_layout.addWidget(self.turn_dir_combo)
        layout.addLayout(turn_dir_layout)
        
        motion_layout = QHBoxLayout()
        self.motion_label = QLabel('Motion Type: ')
        self.motion_text_input = QLineEdit()
        self.motion_text_input.setPlaceholderText('motion (e.g. straight)')
        motion_layout.addWidget(self.motion_label)
        motion_layout.addWidget(self.motion_text_input)
        layout.addLayout(motion_layout)

        folder_select_layout = QHBoxLayout()
        self.selected_folder = '/home/roselab/ros2bags'
        self.folder_label = QLabel(f'Selected folder: {self.selected_folder}')
        self.folder_button = QPushButton('Select Folder')
        self.folder_button.clicked.connect(self.select_folder)
        folder_select_layout.addWidget(self.folder_label)
        folder_select_layout.addWidget(self.folder_button)
        layout.addLayout(folder_select_layout)

        recording_timer_layout = QHBoxLayout()
        self.recording_timer_label = QLabel("Seconds timer: 0")
        recording_timer_layout.addWidget(self.recording_timer_label)
        layout.addLayout(recording_timer_layout)

        record_button_layout = QHBoxLayout()
        self.start_recording_button = QPushButton('Start Recording')
        self.start_recording_button.clicked.connect(self.start_recording)
        record_button_layout.addWidget(self.start_recording_button)

        # self.stop_recording_button = QPushButton('Stop Recording')
        # self.stop_recording_button.clicked.connect(self.stop_recording)
        # record_button_layout.addWidget(self.stop_recording_button)
        layout.addLayout(record_button_layout)

        self.setLayout(layout)
        self.thread_pool = QThreadPool()

    def select_folder(self):
        folder = QFileDialog.getExistingDirectory(self, 'Select Folder')
        if folder:
            self.folder_label.setText(f'Selected folder: \n{folder}')
            self.selected_folder = folder
    
    def start_recording(self):
        async_task = AsyncTask(self.start_recording_coroutine())
        self.thread_pool.start(async_task)
        # rover_driver.execute_trial()
    
    async def start_recording_coroutine(self):
        request = StartBag.Request()
        now = datetime.now()
        # bag name is split up into <motion><slope angle><date time>
        bag_output_dir = f"{self.selected_folder}/{self.motion_text_input.text()}_{self.slope_angle_combo.currentText()}deg_{now.strftime('%Y-%m-%d_%H-%M-%S')}"
        request.bag_output = bag_output_dir

        future = self.start_bag_client.call_async(request)
        rclpy.spin_until_future_complete(self.node_handle, future)  # Wait for the service to respond

        if future.result() is not None:
            # give time for the directory to be recognized for the trial executor
            time.sleep(1)
            response = future.result()
            self.node_handle.get_logger().info(f"Started recording: {'success' if response.success else 'failed'}")
            
            start_trial_request = StartTrial.Request()
            start_trial_request.speed = float(self.speed_combo.currentText()) / 100
            start_trial_request.directory = bag_output_dir
            if self.radius_combo.currentText() == 'infinity':
                start_trial_request.radius = 1000
            else:
                if self.turn_dir_combo.currentText() == 'left':
                    start_trial_request.radius = float(self.radius_combo.currentText()) / 100
                elif self.turn_dir_combo.currentText() == 'right':
                    start_trial_request.radius = float(self.radius_combo.currentText()) / 100 * -1
            start_trial_future = self.start_trial_client.call_async(start_trial_request)
            self.node_handle.get_logger().info(f"Sending trial commands")
            rclpy.spin_until_future_complete(self.node_handle, start_trial_future)
            self.node_handle.get_logger().info(f"Successfully started trial executor")
            for i in range(47):
                sleep(1)
                self.recording_timer_label.setText(f'Trial timer: {i}')
            self.node_handle.get_logger().info(f"Stopping trial")
            self.stop_recording()
        else:
            self.node_handle.get_logger().error('Service call failed')

    def stop_recording(self):
        async_task = AsyncTask(self.stop_recording_coroutine())
        self.thread_pool.start(async_task)
        
    async def stop_recording_coroutine(self):
        request = StopBag.Request()
        future = self.stop_bag_client.call_async(request)
        rclpy.spin_until_future_complete(self.node_handle, future)  # Wait for the service to respond

        if future.result() is not None:
            response = future.result()
            self.node_handle.get_logger().info(f"Stopped recording: {'success' if response.success else 'failed'}")
        else:
            self.node_handle.get_logger().error('Service call failed')

def signal_handler(sig, frame):
    print("Ctrl+c Received. Exiting Trial Menu and Runner")
    QCoreApplication.quit()

def main(args=None):
    app = QApplication(sys.argv)
    s = signal.signal(signal.SIGINT, signal_handler)

    # Start a timer that periodically checks for signals, needed to quit on ctrl+c
    timer = QTimer()
    timer.timeout.connect(lambda: None)  # No-op to keep the event loop responsive
    timer.start(100)  # Check every 100 ms

    bag_builder = BagBuilder()
    bag_builder.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
