#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pplanner_interfaces.msg import PathGrid
from pplanner_interfaces.msg import PathGridset
import threading
import time

class PidPilotSenior(Node):
    def __init__(self):
        super().__init__("pid_pilot_senior") 
        self.get_logger().info("Node has started")
        # self.path_data = self.create_subscription(PathGridset, "path", self.callback_pid, 10)
        self.path_traverse = PathGridset()
        self.path_pid_temp = []
        self.path_pid = [[[0,0],[1,1]]]
        self.j = 0
        self.path_pid_reference = PathGridset() 
        self.temp = PathGrid()
        self.temp.row = 0
        self.temp.col = 0
        self.path_pid_reference.path = [self.temp]
        self.pid_function_local_path = [5,5,5]
        self.path_traverse = [self.temp]
        self.time_interval = 5


    def pid_function(self, path_pid):
        
        while(1):
            if self.pid_function_local_path != path_pid[0]:
                self.get_logger().info("INNNNNNNNNNNNNNNN")
                self.get_logger().info(str(path_pid))
                self.get_logger().info(str(self.pid_function_local_path))

                time.sleep(0.1)



def main(args=None):
    rclpy.init(args=args)
    node = PidPilotSenior()
    rand = [[[0,0],[1,1]]]
    pid_thread = threading.Thread(target = node.pid_function, args = (node.path_pid))
    pid_thread.start()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()
