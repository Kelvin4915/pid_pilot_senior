#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pplanner_interfaces.msg import PathGrid
from pplanner_interfaces.msg import PathGridset
from pplanner_interfaces.msg import ArucoData
from pplanner_interfaces.msg import ArucoDataset
import numpy as np
import threading
import math
import time
import RPi.GPIO as GPIO

class PidPilotSenior(Node):
    def __init__(self):
        super().__init__("pid_pilot_senior") 
        self.get_logger().info("PidPilotSenior has started")

        #---------------CHANGE ME ACCORDING TO ROBOT ID-------------
        self.robot_id = 3 
        """
        --TO--DO--
        set kp,ki,kd values
        add extra filler values
        set time interval
        define equations of motion
        map duty cycle
        gpio commands
        
        """

        self.path_data = self.create_subscription(PathGridset, "path", self.callback_pid_path, 10)
        self.robot_data = self.create_subscription(ArucoDataset, "robots", self.callback_actual_position, 10)
        self.path_traverse = PathGridset()
        self.path_pid_temp = []
        # self.path_pid = [[0,0],[1,3],[5,2],[5,7],[3,1],[2,5]]
        self.path_pid = []
        self.path_pid_reference = PathGridset() 
        self.temp = PathGrid()
        self.temp.row = 0
        self.temp.col = 0
        self.path_pid_reference.path = [self.temp]

        self.all_robots_data = ArucoDataset
        self.actual_position = [0,0,0]
        
        self.pid_function_local_path_reference = []
        self.path_traverse = [self.temp]

        self.position = []
        self.velocity = []
        self.acceleration = []
        
        # PID variables
        self.Kp = 10
        self.Ki = 0
        self.Kd = 0
        self.Kp_theta = 0.5
        self.Ki_theta = 0
        self.Kd_theta = 0
        self.pid_reference_counter = 0
        self.instantaneous_position_data = [[0,0,0],[0,0,0]]
        self.instantaneous_velocity_data = [[0,0,0],[0,0,0]]
        self.instantaneous_acceleration_data = [[0,0,0],[0,0,0]]
        self.corrected_velocity = [0,0,0]
        self.error_timer = [0.0, 0.0]
        self.E = [[0,0,0],[0,0,0]]
        self.E_dot = [0,0,0]
        self.E_adot = [0,0,0]

        # equation of motion varaiables
        self.Va = 0.0
        self.Theta_dot = 0.0
        self.Phi_dot_L = 0.0
        self.Phi_dot_R = 0.0
        self.r = 6.5 # INSERT ACTUAL VALUE (CM)
        self.S = 11.5 # INSERT ACTUAL VALUE (CM)

        self.Phi_dot_L_range_min = -0.68 * 2 * np.pi # INSERT ACTUAL VALUE (Rad/s)
        self.Phi_dot_L_range_max =  0.68 * 2 * np.pi # INSERT ACTUAL VALUE (Rad/s)
        self.Phi_dot_L_range = np.array([self.Phi_dot_L_range_min, self.Phi_dot_L_range_max])

        self.Phi_dot_R_range_min = -0.68 * 2 * np.pi # INSERT ACTUAL VALUE (Rad/s)
        self.Phi_dot_R_range_max =  0.68 * 2 * np.pi # INSERT ACTUAL VALUE (Rad/s)
        self.Phi_dot_R_range = np.array([self.Phi_dot_R_range_min, self.Phi_dot_R_range_max])

        self.L_range_actual_min_rot = 7.4 # INSERT ACTUAL VALUE
        self.L_range_actual_max_rot = 7.6 # INSERT ACTUAL VALUE
        self.L_range_actual_rot = np.array([self.L_range_actual_min_rot, self.L_range_actual_max_rot])

        self.R_range_actual_min_rot = 7.6 # INSERT ACTUAL VALUE
        self.R_range_actual_max_rot = 7.4 # INSERT ACTUAL VALUE
        self.R_range_actual_rot = np.array([self.R_range_actual_min_rot, self.R_range_actual_max_rot])

        self.L_range_actual_min_lin = 6 # INSERT ACTUAL VALUE
        self.L_range_actual_max_lin = 8 # INSERT ACTUAL VALUE
        self.L_range_actual_lin = np.array([self.L_range_actual_min_lin, self.L_range_actual_max_lin])

        self.R_range_actual_min_lin = 8 # INSERT ACTUAL VALUE
        self.R_range_actual_max_lin = 6 # INSERT ACTUAL VALUE
        self.R_range_actual_lin = np.array([self.R_range_actual_min_lin, self.R_range_actual_max_lin])

        self.Phi_dot_L_act_rot = 0.0
        self.Phi_dot_R_act_rot = 0.0
        self.Phi_dot_L_act_lin = 0.0
        self.Phi_dot_R_act_lin = 0.0

        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        self.servo_motor_left_pin = 12 # INSERT ACTUAL VALUE
        self.servo_motor_right_pin = 18 # INSERT ACTUAL VALUE
        GPIO.setup(self.servo_motor_left_pin, GPIO.OUT)
        GPIO.setup(self.servo_motor_right_pin, GPIO.OUT)

        self.servo_motor_left = GPIO.PWM(self.servo_motor_left_pin, 50)
        self.servo_motor_right = GPIO.PWM(self.servo_motor_right_pin, 50)

        self.servo_motor_left.start(7.5)
        self.servo_motor_right.start(7.5)

        self.servo_motor_left.ChangeDutyCycle(0.0)
        self.servo_motor_right.ChangeDutyCycle(0.0)

        self.time_check_start = 0.0
        self.time_check_inter = 0.0
        self.position_check = []

        self.orientation_buffer = 0.3
        self.orientation_check_maximum = 0.0
        self.orientation_check_minimum = 0.0
        self.orientation_reference_maximum = 0.0
        self.orientation_reference_minimum = 0.0
        self.reference_flag = 100
        self.boundary_flag = 100
        self.timercounter = 0
        self.old_time = 0
        self.new_time = 0
        pid_thread = threading.Thread(target = self.pid_function)
        pid_thread.start()

        self.pixel_to_cm  = 119/32 # cm / pixels # INSERT ACTUAL VALUE

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
        
        if self.timercounter == 0:
            self.timercounter = 1
            self.new_time = time.time()
        self.old_time = self.new_time
        self.new_time = time.time()
        rate  = 1/(self.new_time - self.old_time)
        # self.get_logger().info("subscribe rate = " + str(rate))
        # self.get_logger().info("subscribe time difference = " + str(self.new_time - self.old_time))

        # self.get_logger().info("robot data received")
        self.all_robots_data = msg
        for i in range(len(self.all_robots_data.dataset)):
            if self.all_robots_data.dataset[i].id_data == self.robot_id:
                
                self.actual_position = [self.all_robots_data.dataset[i].y_data, self.all_robots_data.dataset[i].x_data, self.all_robots_data.dataset[i].orientation_data]
                # self.get_logger().info("robot data: " + str(self.actual_position))
                break

    def loop_break_condition(self,ref_val):
        #self.get_logger().info("reference flag " + str(self.reference_flag))
        if self.reference_flag != self.pid_reference_counter:
            self.reference_flag = self.pid_reference_counter    
            reference_value = ref_val
            self.boundary_flag = 0
            self.orientation_reference_maximum = reference_value + self.orientation_buffer
            self.orientation_reference_minimum = reference_value - self.orientation_buffer
            if self.orientation_reference_maximum >= 2 * np.pi:
                self.orientation_reference_maximum = self.orientation_reference_maximum - (2 * np.pi)
                self.boundary_flag = 1
            if self.orientation_reference_minimum < 0:
                self.orientation_reference_minimum = self.orientation_reference_minimum + (2 * np.pi)
                self.boundary_flag = 1

        #self.get_logger().info("reference orientation" + str(self.position[self.pid_reference_counter][2]/np.pi))
        #self.get_logger().info("orientation_reference_maximum" + str(self.orientation_reference_maximum))
        #self.get_logger().info("orientation_reference_minimum= " + str(self.orientation_reference_minimum))
        #self.get_logger().info("actual_position[2] = " + str(self.actual_position[2]/np.pi))
        #self.get_logger().info("first loop correcting orientation")
        #self.get_logger().info("x diff" + str(self.path_pid[1][0] - self.path_pid[0][0]))
        #self.get_logger().info("y diff" + str(- self.path_pid[1][1] + self.path_pid[0][1]))

        if self.boundary_flag == 0:
            return (self.actual_position[2] < self.orientation_reference_maximum) and (self.actual_position[2] > self.orientation_reference_minimum)
        else:
            return (self.actual_position[2] < self.orientation_reference_maximum) or (self.actual_position[2] > self.orientation_reference_minimum)


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
                    dx = self.path_pid[i+1][1] - self.path_pid[i][1]
                    dy = self.path_pid[i][0] - self.path_pid[i+1][0]
                    # if (dx == 0):
                    #     if (dy > 0):
                    #         orientation = math.pi / 2
                    #     elif (dy < 0):
                    #         orientation = (-1) * math.pi / 2
                    # elif (dy == 0):
                    #     if (dx > 0):
                    #         orientation = 0
                    #     elif (dx < 0):
                    #         orientation = math.pi
                    # else:

                    orientation = math.atan2(dy, dx)
                    if orientation < 0:
                        orientation += 2 * math.pi

                    self.pid_function_local_path.append([self.path_pid[i][0],self.path_pid[i][1],orientation])
                
                
                
                temp_final_pos = [self.path_pid[len(self.path_pid) - 1][0], self.path_pid[len(self.path_pid) - 1][1], self.pid_function_local_path[len(self.pid_function_local_path) - 1][2]]
                self.pid_function_local_path = self.pid_function_local_path + [temp_final_pos]
                self.position = self.pid_function_local_path[:]
                self.pid_function_local_path = self.pid_function_local_path + [temp_final_pos]

                # self.pid_function_local_path.append(self.pid_function_local_path[len(self.pid_function_local_path) - 1])
                # self.pid_function_local_path.append(self.pid_function_local_path[len(self.pid_function_local_path) - 1])

                # self.get_logger().info(str(self.pid_function_local_path) + " 2 pid_function_local_path")
                # self.get_logger().info(str(len(self.pid_function_local_path)) + " length pid_function_local_path")

                # temp_velocity = [[(self.pid_function_local_path[i+1][0] - self.pid_function_local_path[i][0]) / self.time_interval, 
                #     (self.pid_function_local_path[i+1][1] - self.pid_function_local_path[i][1]) / self.time_interval,
                #     (self.pid_function_local_path[i+1][2] - self.pid_function_local_path[i][2]) / self.time_interval] for i in range(0, len(self.pid_function_local_path)-1)]
                
                # self.velocity = temp_velocity
                # temp_velocity = temp_velocity + [[0,0,0]]

                # self.acceleration = [[(temp_velocity[i+1][0] - temp_velocity[i][0]) / self.time_interval,
                #      (temp_velocity[i+1][1] - temp_velocity[i][1]) / self.time_interval,
                #      (temp_velocity[i+1][2] - temp_velocity[i][2]) / self.time_interval] for i in range(0, len(temp_velocity)-1)]
                
                # self.get_logger().info(str(len(path_pid)) + " length path_pid")

                self.get_logger().info(str(self.position) + " position")
                self.get_logger().info(str(len(self.position)) + " length position")

                #self.get_logger().info(str(self.velocity) + " velocity")
                #self.get_logger().info(str(len(self.velocity)) + " length velocity")

                #self.get_logger().info(str(self.acceleration) + " acceleration")
                #self.get_logger().info(str(len(self.acceleration)) + " length acceleration")

            if self.pid_function_local_path_reference == self.path_pid and self.pid_reference_counter <= len(self.position) - 1:
                if len(self.path_pid) != 0:

                    self.get_logger().info("Inside Controller Loop")
                    if self.pid_reference_counter == 0:    
                        self.instantaneous_position_data = [self.actual_position,self.actual_position] 

                    self.instantaneous_position_data[0] = self.instantaneous_position_data[1]
                    self.instantaneous_position_data[1] = self.actual_position
                    
                    self.error_timer[0] = self.error_timer[1]
                    self.error_timer[1] = time.time()
                    self.error_timer_diff = self.error_timer[1] - self.error_timer[0]

                    self.E[0] = self.E[1]
                    self.E[1] = [self.position[self.pid_reference_counter][0] - self.instantaneous_position_data[1][0],
                                self.position[self.pid_reference_counter][1] - self.instantaneous_position_data[1][1],
                                self.position[self.pid_reference_counter][2] - self.instantaneous_position_data[1][2]]
                    
                    if self.E[1][2] > np.pi:
                        self.E[1][2] = self.E[1][2] - (2 * np.pi)
                    
                    if self.E[1][2] < -np.pi:
                        self.E[1][2] = self.E[1][2] + (2 * np.pi)
                    
                    self.E_dot = [((self.E[1][0] - self.E[0][0]) / self.error_timer_diff),
                                    ((self.E[1][1] - self.E[0][1]) / self.error_timer_diff),
                                    ((self.E[1][2] - self.E[0][2]) / self.error_timer_diff)]
                    
                    self.E_adot = [self.E_adot[0] + (0.5 * (self.E[1][0] + self.E[0][0]) * self.error_timer_diff),
                                    self.E_adot[1] + (0.5 * (self.E[1][1] + self.E[0][1]) * self.error_timer_diff),
                                    self.E_adot[2] + (0.5 * (self.E[1][2] + self.E[0][2]) * self.error_timer_diff)]

                    PID_velocity = [self.E[1][0]*self.Kp + self.E_dot[0]*self.Kd + self.E_adot[0]*self.Ki,
                                    self.E[1][1]*self.Kp + self.E_dot[1]*self.Kd + self.E_adot[1]*self.Ki,
                                    self.E[1][2]*self.Kp_theta + self.E_dot[2]*self.Kd_theta+ self.E_adot[2]*self.Ki_theta]

                    # self.get_logger().info("PID_velocity = " + str(PID_velocity))
                    # time.sleep(1)

                    self.Va = self.pixel_to_cm * (math.sqrt((PID_velocity[0])**2 + (PID_velocity[1])**2 ))
                    #self.Va = 0
                    self.Theta_dot = PID_velocity[2]

                    self.Phi_dot_L = (1/self.r)*(self.Va - ((self.Theta_dot * self.S)/2))
                    self.Phi_dot_R = (1/self.r)*(self.Va + ((self.Theta_dot * self.S)/2))

                    self.time_check_start = time.time()
                    self.time_check_inter = time.time()
                    self.position_check = self.actual_position[:1]
                    self.orientation_check_maximum = self.actual_position[2] + self.orientation_buffer
                    self.orientation_check_minimum = self.actual_position[2] - self.orientation_buffer
                    if self.orientation_check_maximum >= 2 * np.pi:
                        self.orientation_check_maximum = self.orientation_check_maximum - (2 * np.pi)
                    if self.orientation_check_minimum < 0:
                        self.orientation_check_minimum = self.orientation_check_minimum + (2 * np.pi)
                    
                    self.get_logger().info("counter number " + str(self.pid_reference_counter))
                    self.get_logger().info("Va = " + str(self.Va))
                    self.get_logger().info("Theta_dot = " + str(self.Theta_dot))
                    self.get_logger().info("instantaneous_position_data = " + str(self.instantaneous_position_data))
                    self.get_logger().info("x error = " + str(self.E[1][0]))
                    self.get_logger().info("y error = " + str(self.E[1][1]))
                    self.get_logger().info("theta error = " + str(self.E[1][2]))
                    self.get_logger().info("theta dot = " + str(self.Theta_dot))
                    self.get_logger().info("PHI dot L = " + str(self.Phi_dot_L))
                    self.get_logger().info("PHI dot R = " + str(self.Phi_dot_R))
                    self.get_logger().info("DUTY cycle LEFT = " + str(self.Phi_dot_L_act))
                    self.get_logger().info("DUTY cycle RIGHT = " + str(self.Phi_dot_R_act))


                    if ((self.actual_position[:1] == self.position_check)):
                        self.get_logger().info(" position check passed")

                    if (-0.5  >= self.E[1][2] or self.E[1][2]>= 0.5):
                        self.get_logger().info("orientation check passed")
                    self.get_logger().info(" E[1][2] = " + str(self.E[1][2]))

                    if self.pid_reference_counter > 0:
                        self.Phi_dot_L_act_lin = np.interp(self.Phi_dot_L, self.Phi_dot_L_range, self.L_range_actual_lin)
                        self.Phi_dot_R_act_lin = np.interp(self.Phi_dot_R, self.Phi_dot_R_range, self.R_range_actual_lin)


                        self.servo_motor_left.ChangeDutyCycle(self.Phi_dot_L_act_lin)
                        self.servo_motor_right.ChangeDutyCycle(self.Phi_dot_R_act_lin)
                    
                        #pass
                        #while(((self.time_check_inter - self.time_check_start) < 5) and ( self.actual_position[:1] == self.position_check ) and (self.loop_break_condition(self.actual_position[2]))):
                        while(self.actual_position[:1] == self.position_check):
                            self.time_check_inter = time.time()
                            #self.get_logger().info(" inside while loop")


                    if self.pid_reference_counter == 0:  
                        self.Phi_dot_L_act_rot = np.interp(self.Phi_dot_L, self.Phi_dot_L_range, self.L_range_actual_rot)
                        self.Phi_dot_R_act_rot = np.interp(self.Phi_dot_R, self.Phi_dot_R_range, self.R_range_actual_rot)


                        self.servo_motor_left.ChangeDutyCycle(self.Phi_dot_L_act_rot)
                        self.servo_motor_right.ChangeDutyCycle(self.Phi_dot_R_act_rot)
                    
                        # self.orientation_reference_maximum = self.position[self.pid_reference_counter][2] + self.orientation_buffer
                        # self.orientation_reference_minimum = self.position[self.pid_reference_counter][2] - self.orientation_buffer
                        # if self.orientation_reference_maximum >= 2 * np.pi:
                        #     self.orientation_reference_maximum = self.orientation_reference_maximum - (2 * np.pi)
                        # if self.orientation_reference_minimum < 0:
                        #     self.orientation_reference_minimum = self.orientation_reference_minimum + (2 * np.pi)

                        while(not self.loop_break_condition(self.position[self.pid_reference_counter][2])):
                        #while(not (self.orientation_reference_maximum > self.actual_position[2] or self.actual_position[2] > (self.orientation_reference_minimum))):
                            self.get_logger().info("reference orientation" + str(self.position[self.pid_reference_counter][2]/np.pi))
                            #self.get_logger().info("orientation_reference_maximum" + str(self.orientation_reference_maximum))
                            #self.get_logger().info("orientation_reference_minimum= " + str(self.orientation_reference_minimum))
                            self.get_logger().info("actual_position[2] = " + str(self.actual_position[2]/np.pi))
                            #self.get_logger().info("first loop correcting orientation")
                            self.get_logger().info("x diff" + str(self.path_pid[1][0] - self.path_pid[0][0]))
                            self.get_logger().info("y diff" + str(- self.path_pid[1][1] + self.path_pid[0][1]))

                    self.pid_reference_counter = self.pid_reference_counter + 1

            # self.get_logger().info("position " + str(len(self.position)))
            # self.get_logger().info("velocity " + str(len(self.velocity)))
            # self.get_logger().info("acceleration " + str(len(self.acceleration)))
            self.servo_motor_left.ChangeDutyCycle(0)
            self.servo_motor_right.ChangeDutyCycle(0)

def main(args=None):
    rclpy.init(args=args)
    node = PidPilotSenior()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()
