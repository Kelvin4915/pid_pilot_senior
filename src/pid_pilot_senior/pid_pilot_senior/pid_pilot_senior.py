#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pplanner_interfaces.msg import PathGrid
from pplanner_interfaces.msg import PathGridset
import threading
import math

class PidPilotSenior(Node):
    def __init__(self):
        super().__init__("pid_pilot_senior") 
        self.get_logger().info("Node has started")
        # self.path_data = self.create_subscription(PathGridset, "path", self.callback_pid, 10)
        self.path_traverse = PathGridset()
        self.path_pid_temp = []
        self.path_pid = [[[0,0],[1,3],[5,2],[5,7],[3,1],[2,5]]]
        self.j = 0
        self.path_pid_reference = PathGridset() 
        self.temp = PathGrid()
        self.temp.row = 0
        self.temp.col = 0
        self.path_pid_reference.path = [self.temp]
        self.pid_function_local_path = []
        self.pid_function_local_path_reference = []
        self.path_traverse = [self.temp]
        self.time_interval = 5

        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.pid_reference_counter = 0
        self.instantaneous_position_data = [[0,0],[0,0]]
        self.instantaneous_velocity_data = [[0,0],[0,0]]
        self.instantaneous_acceleration_data = [[0,0],[0,0]]
        self.corrected_velocity = [0,0]

        pid_thread = threading.Thread(target = self.pid_function, args = (self.path_pid))
        pid_thread.start()

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
            if self.pid_function_local_path_reference != path_pid:
                self.get_logger().info("Inside Calculation Loop")
                self.pid_reference_counter = 1

                self.get_logger().info(str(self.pid_function_local_path) + " 1 pid_function_local_path")
                self.get_logger().info(str(len(path_pid)) + " length path_pid")
                
                self.pid_function_local_path_reference = path_pid[:]
                self.pid_function_local_path = path_pid[:]
                position = path_pid[:]
                temp_init_pos = self.pid_function_local_path[0]
                self.pid_function_local_path = [temp_init_pos] + self.pid_function_local_path
                # self.pid_function_local_path.append(self.pid_function_local_path[len(self.pid_function_local_path) - 1])
                # self.pid_function_local_path.append(self.pid_function_local_path[len(self.pid_function_local_path) - 1])

                self.get_logger().info(str(self.pid_function_local_path) + " 2 pid_function_local_path")
                self.get_logger().info(str(len(self.pid_function_local_path)) + " length pid_function_local_path")

                temp_velocity = [[(self.pid_function_local_path[i+1][0] - self.pid_function_local_path[i][0]) / self.time_interval, 
                    (self.pid_function_local_path[i+1][1] - self.pid_function_local_path[i][1]) / self.time_interval] for i in range(0, len(self.pid_function_local_path)-1)]
                
                velocity = temp_velocity
                temp_velocity = [[0,0]] + temp_velocity

                acceleration = [[(temp_velocity[i+1][0] - temp_velocity[i][0]) / self.time_interval,
                     (temp_velocity[i+1][1] - temp_velocity[i][1]) / self.time_interval] for i in range(0, len(temp_velocity)-1)]
                
                # self.get_logger().info(str(len(path_pid)) + " length path_pid")

                self.get_logger().info(str(position) + " position")
                self.get_logger().info(str(len(position)) + " length position")

                self.get_logger().info(str(velocity) + " velocity")
                self.get_logger().info(str(len(velocity)) + " length velocity")

                self.get_logger().info(str(acceleration) + " acceleration")
                self.get_logger().info(str(len(acceleration)) + " length acceleration")

            if self.pid_function_local_path_reference == path_pid:
                self.get_logger().info("Inside Controller Loop")
                if self.pid_reference_counter == 1:    
                    self.instantaneous_position_data = [actual_position,actual_position] 

                self.instantaneous_position_data[0] = self.instantaneous_position_data[1]
                self.instantaneous_position_data[1] = actual_position

                self.instantaneous_velocity_data[0] = self.instantaneous_velocity_data[1]
                self.instantaneous_velocity_data[1] = (self.instantaneous_position_data[1] - self.instantaneous_position_data[0])/self.time_interval

                self.instantaneous_acceleration_data[0] = self.instantaneous_acceleration_data[1]
                self.instantaneous_acceleration_data[1] = (self.instantaneous_velocity_data[1] - self.instantaneous_velocity_data[0])/self.time_interval
            
                Ep = position[self.pid_reference_counter] - self.instantaneous_position_data[1]
                Ev = velocity[self.pid_reference_counter] - self.instantaneous_velocity_data[1]
                Ea = acceleration[self.pid_reference_counter] - self.instantaneous_acceleration_data[1]

                PID_velocity = Ep*self.Ki + Ev*self.Kp + Ea*self.Kd
                self.corrected_velocity = self.corrected_velocity + PID_velocity


                self.pid_reference_counter = self.pid_reference_counter + 1



            # self.get_logger().info("position" + str(len(self.pid_function_local_path)))
            # self.get_logger().info("velocity" + str(len(velocity)))
            # self.get_logger().info("acceleration" + str(len(acceleration)))


def main(args=None):
    rclpy.init(args=args)
    node = PidPilotSenior()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()
