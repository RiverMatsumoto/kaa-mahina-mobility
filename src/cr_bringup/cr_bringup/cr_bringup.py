#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2node.api import get_node_names

import subprocess
import threading
import curses
import signal

class BringupNode(Node):
    def __init__(self, screen):
        super().__init__('bringup_node')
        self.get_logger().info('Bringup Node has been initialized')
        self.screen = curses.initscr()
        curses.curs_set(0)
        self.screen.nodelay(True)
        self.last_key = 'None'

        self.launch_processes: dict[str, subprocess.Popen] = dict()

        self.status_bar = ''

        # Start curses display thread
        self.curses_thread = threading.Thread(target=self.display, daemon=True)
        self.curses_thread.start()

    def destroy_node(self):
        for launch_name, process in self.launch_processes.items():
            process.send_signal(signal.SIGINT)
            process.wait()
            print(f"Killed {launch_name}")
        super().destroy_node()
    
    def get_alive_nodes(self) -> list[str]:
        process = subprocess.run(["ros2", "node", "list"], capture_output=True)
        # for each line, check node name
        return [s.decode('utf-8') for s in process.stdout.splitlines()]

    def update_process_status(self):
        launch_processes_keys = self.launch_processes.copy().keys()
        for key in launch_processes_keys:
            if self.launch_processes[key].poll() is not None:
                self.launch_processes.pop(key)
    
    def display_launch_scheme(self, 
                            key_pressed: int,
                            key_to_launch: int, 
                            key_to_kill: int,
                            launch_name: str,
                            row: int,
                            launch_cmd: list[str]):
        self.screen.addstr(row, 0, f"Press {chr(key_to_launch)} to launch {launch_name}")
        self.screen.addstr(row, 60, f"Status: {'Alive' if launch_name in self.alive_nodes else 'Dead'}")
        if key_pressed == key_to_launch:
            if launch_name not in self.launch_processes.keys(): 
                self.launch_processes[launch_name] = subprocess.Popen(launch_cmd, 
                                                        stdout=subprocess.PIPE, 
                                                        stderr=subprocess.PIPE,
                                                        text=True)
                self.status_bar = f"Launching {launch_name}"
            else:
                self.status_bar = f"Tried launching {launch_name}, but it's already running"
        if key_pressed == key_to_kill:
            if launch_name in self.launch_processes.keys(): 
                self.launch_processes[launch_name].send_signal(signal.SIGINT)
                self.status_bar = f"Killing {launch_name}"
            else:
                self.status_bar = f"Tried killing {launch_name}, but it's not running"
        
    
    def display(self):
        while True:
            key = self.screen.getch()
            if key == 27: # ESC key
                curses.endwin()
                self.get_logger().info("Shutting down bringup_node, waiting for child launch processes to finish")
                rclpy.shutdown()

            if not rclpy.ok():
                break
            self.screen.clear()

            # list out alive nodes
            self.update_process_status()
            self.alive_nodes = get_node_names(node=self)
            self.alive_nodes = list(map(lambda x: x[1] + x[0], self.alive_nodes))
            
            # launch mappings
            self.display_launch_scheme(
                key_pressed=key,
                key_to_launch=ord('1'),
                key_to_kill=ord('!'),
                launch_name="differential_drive.launch.py",
                row=3,
                launch_cmd=["ros2", "launch", "cr_hardware", "differential_drive.launch.py"]
            )
            self.display_launch_scheme(
                key_pressed=key,
                key_to_launch=ord('2'),
                key_to_kill=ord('@'),
                launch_name="bno055.launch.py",
                row=4,
                launch_cmd=["ros2", "launch", "cr_hardware", "bno055.launch.py"]
            )
                    
            # status bar
            self.screen.addstr(10, 0, f"Status: {self.status_bar}")

            self.screen.addstr(0, 0, f"Press ESC to exit")
            self.screen.addstr(1, 0, f"Key pressed: {chr(key) if key != -1 else self.last_key}")
            self.last_key = chr(key) if key != -1 else self.last_key
            self.screen.addstr(2, 0, f"Press shift+<num_key> to kill the node")
            self.screen.addstr(0, 80, "Alive Nodes")
            self.screen.addstr(1, 80, "===========")
            for i, node_name in enumerate(self.alive_nodes):
                self.screen.addstr(i+2, 80, node_name)
            
            self.screen.refresh()
            curses.napms(200)

def main(args=None):
    rclpy.init()
    # set up sigint callback before creating node and running curses
    def run_curses(screen):
        node = BringupNode(screen)
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            curses.endwin()
            exit(0)
    curses.wrapper(run_curses)
        
if __name__ == "__main__":
    main()