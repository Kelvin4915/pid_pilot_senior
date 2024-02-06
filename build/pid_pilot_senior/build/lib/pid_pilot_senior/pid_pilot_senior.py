#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pplanner_interfaces.msg import PathGrid
from pplanner_interfaces.msg import PathGridset

class PidPilotSenior(Node):
    def __init__(self):
        super().__init__("pid_pilot_senior") 
        self.get_logger().info("Node has started")
        self.path_data = self.create_subscription(PathGridset, "path", self.callback_pid, 10)
        self.path_traverse = PathGridset()
        self.var2 = []
        self.var3 = []
        self.j = 0
        self.var4 = PathGridset() 
        self.temp = PathGrid()
        self.temp.row = 0
        self.temp.col = 0

        self.var4.path = [self.temp]


    def callback_pid(self,msg):
        self.get_logger().info("data received")
        var = PathGridset()
        var = msg
        self.path_traverse = msg
        #self.get_logger().info(str(self.path_traverse))

        if self.path_traverse != self.var4:
            self.var3 = []
            for i in range(len(self.path_traverse.path)):            
                self.var2 = [self.path_traverse.path[i].row,self.path_traverse.path[i].col]
                self.var3.append(self.var2)
            self.j = 0

        if self.j==0:    
            self.var4 = self.path_traverse
            self.j=1

        self.get_logger().info(str(self.var3))

        



def main(args=None):
    rclpy.init(args=args)
    node = PidPilotSenior()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()
