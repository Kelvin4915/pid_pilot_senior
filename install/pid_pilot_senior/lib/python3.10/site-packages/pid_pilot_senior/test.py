#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
import threading
import time

v = 0  # Declare global variable outside of functions

# class TestingClass(Node)
#     def __init__(self):
ran = ["HalalCart"]
justK = ran
def task1():
    global v  # Access global variable within the function
    while True:
        print("Task 1 running on thread ", v)
        time.sleep(0.5)
        v += 2

def task2(kilos):
    global v  # Access global variable within the function

    while True:
        print("Task 2 running on thread", v)
        print(kilos)
        
        time.sleep(0.5)
        if v == 20: 
            v = 0
            
# Create threads
thread1 = threading.Thread(target=task1)
thread2 = threading.Thread(target=task2 , args = justK)

# Start threads
thread1.start()
thread2.start()

# def main(args=None):
#     rclpy.init(args=args)
#     node = PidPilotSenior()
#     rclpy.spin(node)
#     rclpy.shutdown()
 
# if __name__ == "__main__":
#     main()
