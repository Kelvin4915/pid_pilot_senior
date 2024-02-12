#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pplanner_interfaces.msg import PathGrid
from pplanner_interfaces.msg import PathGridset
import threading

class PidPilotSenior(Node):
    def __init__(self):
        super().__init__("pid_pilot_senior") 
        self.get_logger().info("Node has started")
        self.path_data = self.create_subscription(PathGridset, "path", self.callback_pid, 10)
        self.path_traverse = PathGridset()
        self.path_pid_temp = []
        self.path_pid = []
        self.j = 0
        self.path_pid_reference = PathGridset() 
        self.temp = PathGrid()
        self.temp.row = 0
        self.temp.col = 0
        self.path_pid_reference.path = [self.temp]
        #self.var3 = [[5, 19], [6, 18], [7, 17], [8, 16], [9, 15], [10, 14], [11, 13], [12, 12], [12, 11], [12, 10], [12, 9]]
        #self.get_logger().info(str(self.var3))
        # self.counter = 0
        # self.counter2 = 0
        pid_thread = threading.Thread(target = self.pid_function)
        pid_thread.start()

    def callback_pid(self,msg):
        self.get_logger().info("data received")
        # var = PathGridset()
        # var = msg
        self.path_traverse = msg
        #self.get_logger().info(str(self.path_traverse))

        if self.path_traverse != self.path_pid_reference:
            #self.var3 = []
            for i in range(len(self.path_traverse.path)):            
                self.path_pid_temp = [self.path_traverse.path[i].row,self.path_traverse.path[i].col]
                self.path_pid.append(self.path_pid_temp)
            self.j = 0

        if self.j==0:    
            self.path_pid_reference = self.path_traverse
            self.j=1

        self.get_logger().info(str(self.path_pid))
        # self.counter = self.counter + 1
        # self.get_logger().info("outside" + str(self.counter))
        # while(1):
        #     self.counter2 = self.counter2 + 1
        #     self.get_logger().info("inside" + str(self.counter2))

    def pid_function():
        pass

        



def main(args=None):
    rclpy.init(args=args)
    node = PidPilotSenior()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()
