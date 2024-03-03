#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pplanner_interfaces.msg import PathGrid
from pplanner_interfaces.msg import PathGridset
from pplanner_interfaces.msg import ArucoData
from pplanner_interfaces.msg import ArucoDataset
import threading
import math

class PidPilotSenior(Node):
    def __init__(self):
        super().__init__("pid_pilot_senior") 
        self.get_logger().info("PidPilotSenior has started")
        self.path_data = self.create_subscription(PathGridset, "path", self.callback_pid_path, 10)
        # self.path_data = self.create_subscription(ArucoDataset, "robots", self.callback_actual_position, 10)
        self.path_traverse = PathGridset()
        self.path_pid_temp = []
        self.path_pid = [[0,0],[1,3],[5,2],[5,7],[3,1],[2,5]]
        self.path_pid_reference = PathGridset() 
        self.temp = PathGrid()
        self.temp.row = 0
        self.temp.col = 0
        self.path_pid_reference.path = [self.temp]
        
        self.pid_function_local_path_reference = []
        self.path_traverse = [self.temp]
        self.time_interval = 5

        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.pid_reference_counter = 0
        self.instantaneous_position_data = [[0,0,0],[0,0,0]]
        self.instantaneous_velocity_data = [[0,0,0],[0,0,0]]
        self.instantaneous_acceleration_data = [[0,0,0],[0,0,0]]
        self.corrected_velocity = [0,0,0]

        pid_thread = threading.Thread(target = self.pid_function)
        pid_thread.start()

    def callback_pid_path(self,msg):
        self.get_logger().info("path data received")
        self.path_traverse = msg
        # self.get_logger().info("self.path_traverse: " + str(self.path_traverse))
        if (self.path_pid_reference != self.path_traverse):
            self.path_pid_reference = self.path_traverse
            self.path_pid_temp = []
            for i in range(len(self.path_traverse.path)):            
                self.path_pid_temp.append([self.path_traverse.path[i].row,self.path_traverse.path[i].col])
            self.path_pid = self.path_pid_temp[:]
            self.get_logger().info("self.path_pid: " + str(self.path_pid))

    def callback_actual_position(self, msg):
        self.get_logger().info("robot data received")

    def pid_function(self):
        
        while(1):
            # self.get_logger().info(str(self.path_pid))
            if self.pid_function_local_path_reference != self.path_pid:
                self.get_logger().info("Inside Calculation Loop")
                self.pid_reference_counter = 0

                # self.get_logger().info(str(self.pid_function_local_path) + " 1 pid_function_local_path")
                # self.get_logger().info(str(len(path_pid)) + " length path_pid")

                self.pid_function_local_path_reference = self.path_pid[:]
                # self.pid_function_local_path = path_pid[0: len(path_pid) - 2]
                self.pid_function_local_path = []
                for i in range(0, len(self.path_pid) - 1 ):
                    dx = self.path_pid[i+1][0] - self.path_pid[i][0]
                    dy = self.path_pid[i+1][1] - self.path_pid[i][1]
                    if (dx == 0):
                        if (dy > 0):
                            orientation = math.pi / 2
                        elif (dy < 0):
                            orientation = (-1) * math.pi / 2
                    elif (dy == 0):
                        if (dx > 0):
                            orientation = 0
                        elif (dx < 0):
                            orientation = math.pi
                    else:
                        orientation = math.atan(dy / dx)
                    self.pid_function_local_path.append([self.path_pid[i][0],self.path_pid[i][1],orientation])
                
                
                
                temp_final_pos = [self.path_pid[len(self.path_pid) - 1][0], self.path_pid[len(self.path_pid) - 1][1], self.pid_function_local_path[len(self.pid_function_local_path) - 1][2]]
                self.pid_function_local_path = self.pid_function_local_path + [temp_final_pos]
                position = self.pid_function_local_path[:]
                self.pid_function_local_path = self.pid_function_local_path + [temp_final_pos]

                # self.pid_function_local_path.append(self.pid_function_local_path[len(self.pid_function_local_path) - 1])
                # self.pid_function_local_path.append(self.pid_function_local_path[len(self.pid_function_local_path) - 1])

                # self.get_logger().info(str(self.pid_function_local_path) + " 2 pid_function_local_path")
                # self.get_logger().info(str(len(self.pid_function_local_path)) + " length pid_function_local_path")

                temp_velocity = [[(self.pid_function_local_path[i+1][0] - self.pid_function_local_path[i][0]) / self.time_interval, 
                    (self.pid_function_local_path[i+1][1] - self.pid_function_local_path[i][1]) / self.time_interval,
                    (self.pid_function_local_path[i+1][2] - self.pid_function_local_path[i][2]) / self.time_interval] for i in range(0, len(self.pid_function_local_path)-1)]
                
                velocity = temp_velocity
                temp_velocity = temp_velocity + [[0,0,0]]

                acceleration = [[(temp_velocity[i+1][0] - temp_velocity[i][0]) / self.time_interval,
                     (temp_velocity[i+1][1] - temp_velocity[i][1]) / self.time_interval,
                     (temp_velocity[i+1][2] - temp_velocity[i][2]) / self.time_interval] for i in range(0, len(temp_velocity)-1)]
                
                # self.get_logger().info(str(len(path_pid)) + " length path_pid")

                self.get_logger().info(str(position) + " position")
                self.get_logger().info(str(len(position)) + " length position")

                self.get_logger().info(str(velocity) + " velocity")
                self.get_logger().info(str(len(velocity)) + " length velocity")

                self.get_logger().info(str(acceleration) + " acceleration")
                self.get_logger().info(str(len(acceleration)) + " length acceleration")

            if self.pid_function_local_path_reference == self.path_pid and self.pid_reference_counter <= len(position) - 1:
                self.get_logger().info("Inside Controller Loop")
                actual_position = [0,0,0]
                if self.pid_reference_counter == 0:    
                    self.instantaneous_position_data = [actual_position,actual_position] 

                self.instantaneous_position_data[0] = self.instantaneous_position_data[1]
                self.instantaneous_position_data[1] = actual_position

                self.instantaneous_velocity_data[0] = self.instantaneous_velocity_data[1]
                self.instantaneous_velocity_data[1] = [(self.instantaneous_position_data[1][0] - self.instantaneous_position_data[0][0])/self.time_interval,
                                                       (self.instantaneous_position_data[1][1] - self.instantaneous_position_data[0][1])/self.time_interval,
                                                       (self.instantaneous_position_data[1][2] - self.instantaneous_position_data[0][2])/self.time_interval,]

                self.instantaneous_acceleration_data[0] = self.instantaneous_acceleration_data[1]
                self.instantaneous_acceleration_data[1] = [(self.instantaneous_velocity_data[1][0] - self.instantaneous_velocity_data[0][0])/self.time_interval,
                                                           (self.instantaneous_velocity_data[1][0] - self.instantaneous_velocity_data[0][0])/self.time_interval,
                                                           (self.instantaneous_velocity_data[1][0] - self.instantaneous_velocity_data[0][0])/self.time_interval]
            
                Ep = [position[self.pid_reference_counter][0] - self.instantaneous_position_data[1][0],
                      position[self.pid_reference_counter][1] - self.instantaneous_position_data[1][1],
                      position[self.pid_reference_counter][2] - self.instantaneous_position_data[1][2]]
                
                Ev = [velocity[self.pid_reference_counter][0] - self.instantaneous_velocity_data[1][0],
                      velocity[self.pid_reference_counter][1] - self.instantaneous_velocity_data[1][1],
                      velocity[self.pid_reference_counter][2] - self.instantaneous_velocity_data[1][2]]
                
                Ea = [acceleration[self.pid_reference_counter][0] - self.instantaneous_acceleration_data[1][0],
                      acceleration[self.pid_reference_counter][1] - self.instantaneous_acceleration_data[1][1],
                      acceleration[self.pid_reference_counter][2] - self.instantaneous_acceleration_data[1][2]]

                PID_velocity = [Ep[0]*self.Ki + Ev[0]*self.Kp + Ea[0]*self.Kd,
                                Ep[1]*self.Ki + Ev[1]*self.Kp + Ea[1]*self.Kd,
                                Ep[2]*self.Ki + Ev[2]*self.Kp + Ea[2]*self.Kd]

                self.corrected_velocity = [self.instantaneous_velocity_data[1][0] + PID_velocity[0],
                                           self.instantaneous_velocity_data[1][1] + PID_velocity[1],
                                           self.instantaneous_velocity_data[1][2] + PID_velocity[2]]

                self.get_logger().info("PID_velocity = " + str(PID_velocity))
            
                self.pid_reference_counter = self.pid_reference_counter + 1

            # self.get_logger().info("position " + str(len(position)))
            # self.get_logger().info("velocity " + str(len(velocity)))
            # self.get_logger().info("acceleration " + str(len(acceleration)))


def main(args=None):
    rclpy.init(args=args)
    node = PidPilotSenior()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()
