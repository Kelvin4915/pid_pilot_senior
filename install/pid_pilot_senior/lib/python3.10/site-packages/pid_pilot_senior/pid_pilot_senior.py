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
        # self.path_data = self.create_subscription(PathGridset, "path", self.callback_pid, 10)
        self.path_traverse = PathGridset()
        self.path_pid_temp = []
        self.path_pid = [[0,0],[1,1]]
        self.j = 0
        self.path_pid_reference = PathGridset() 
        self.temp = PathGrid()
        self.temp.row = 0
        self.temp.col = 0
        self.path_pid_reference.path = [self.temp]
        self.pid_function_local_path = []
        self.path_traverse = [self.temp]
        self.time_interval = 5
        # pid_thread = threading.Thread(target = self.pid_function, args = (self.path_pid))
        # pid_thread.start()

    # def callback_pid(self,msg):
    #     self.get_logger().info("data received")
    #     self.path_traverse = msg
    #     #self.get_logger().info(str(self.path_traverse))

    #     if self.path_traverse != self.path_pid_reference:
    #         self.path_pid_reference = self.path_traverse
    #         for i in range(len(self.path_traverse.path)):            
    #             self.path_pid_temp = [self.path_traverse.path[i].row,self.path_traverse.path[i].col]
    #             self.path_pid.append(self.path_pid_temp)
    #         self.j = 0
    #         self.pid_function(self.path_pid)

    #     self.get_logger().info(str(self.path_pid))

    def pid_function(self, path_pid):
        
        while(1):
            if self.pid_function_local_path != path_pid:
                self.pid_function_local_path = path_pid
                velocity = [[(self.pid_function_local_path[i+1][0] - self.pid_function_local_path[i][0]) / self.time_interval, 
                    (self.pid_function_local_path[i+1][1] - self.pid_function_local_path[i][1]) / self.time_interval] for i in range(0, len(self.pid_function_local_path.path)-1)]

                acceleration = [[(velocity[i+1][0] - velocity[i][0]) / self.time_interval,
                     (velocity[i+1][1] - velocity[i][1]) / self.time_interval] for i in range(0, len(velocity)-1)]
                
                self.get_logger().info(str(velocity))
                self.get_logger().info(str(acceleration))

            self.get_logger().info("position" + str(len(self.pid_function_local_path)))
            self.get_logger().info("velocity" + str(len(velocity)))
            self.get_logger().info("acceleration" + str(len(acceleration)))


def main(args=None):
    rclpy.init(args=args)
    node = PidPilotSenior()
    pid_thread = threading.Thread(target = node.pid_function, args = (node.path_pid))
    pid_thread.start()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()
