#!/usr/bin/env python3
import sys
import threading
import asyncio
from datetime import date
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLineEdit, QPushButton, QCheckBox, QLabel, QFileDialog, QComboBox, QHBoxLayout
from PyQt5.QtCore import QRunnable, pyqtSlot, QThreadPool
import rclpy.service
import rclpy
from cr_bag.srv import StartBag, StopBag

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
        self.setGeometry(100, 100, 1400, 1400)
        
        rclpy.init()  # Initialize the ROS client library
        self.node_handle = rclpy.create_node('bag_builder')  # Create a temporary node
        self.start_bag_client = self.node_handle.create_client(StartBag, 'start_recording')  # Replace 'add_two_ints' with your service name
        self.stop_bag_client = self.node_handle.create_client(StopBag, 'stop_recording')  # Replace 'add_two_ints' with your service name

        layout = QVBoxLayout()

        slope_angle_layout = QHBoxLayout()
        self.slope_angle_label = QLabel('Slope angle: ')
        self.slope_angle_combo = QComboBox(self)
        self.slope_angle_combo.addItems(['0', '5', '10', '15', '20', '25', '30'])
        slope_angle_layout.addWidget(self.slope_angle_label)
        slope_angle_layout.addWidget(self.slope_angle_combo)
        layout.addLayout(slope_angle_layout)
        
        motion_layout = QHBoxLayout()
        self.motion_label = QLabel('Motion Type: ')
        self.motion_text_input = QLineEdit()
        self.motion_text_input.setPlaceholderText('motion (e.g. straight)')
        motion_layout.addWidget(self.motion_label)
        motion_layout.addWidget(self.motion_text_input)
        layout.addLayout(motion_layout)

        folder_select_layout = QHBoxLayout()
        self.folder_label = QLabel('Selected folder:')
        self.folder_button = QPushButton('Select Folder')
        self.folder_button.clicked.connect(self.select_folder)
        folder_select_layout.addWidget(self.folder_label)
        folder_select_layout.addWidget(self.folder_button)
        layout.addLayout(folder_select_layout)

        self.button = QPushButton('Start Recording')
        self.button.clicked.connect(self.start_recording)
        layout.addWidget(self.button)
        self.button = QPushButton('Stop Recording')
        self.button.clicked.connect(self.stop_recording)
        layout.addWidget(self.button)

        self.setLayout(layout)
        self.thread_pool = QThreadPool()

    def select_folder(self):
        folder = QFileDialog.getExistingDirectory(self, 'Select Folder')
        if folder:
            self.folder_label.setText(f'Selected folder: {folder}')
            self.selected_folder = folder
    
    def start_recording(self):
        async_task = AsyncTask(self.start_recording_coroutine())
        self.thread_pool.start(async_task)
    
    async def start_recording_coroutine(self):
        request = StartBag.Request()
        today = date.today()
        request.bag_output = f"{self.selected_folder}/{self.motion_text_input.text()}_{self.slope_angle_combo.currentText()}deg_{today.strftime('%b-%d-%Y')}.bag"

        future = self.start_bag_client.call_async(request)
        rclpy.spin_until_future_complete(self.node_handle, future)  # Wait for the service to respond

        if future.result() is not None:
            response = future.result()
            self.node_handle.get_logger().info(f"Started recording: {'success' if response.success else 'failed'}")
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

def main(args=None):
    app = QApplication(sys.argv)
    bag_builder = BagBuilder()
    bag_builder.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
