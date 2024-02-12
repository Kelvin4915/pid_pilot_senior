import threading
import time

v = 0  # Declare global variable outside of functions

def task1():
    global v  # Access global variable within the function
    # while True:
    print("Task 1 running on thread ", v)
    time.sleep(0.5)
    v += 2

def task2():
    global v  # Access global variable within the function
    # while True:
    print("Task 2 running on thread", v)
    time.sleep(0.5)
    if v == 20: 
        v = 0
        
        
# Create threads
thread1 = threading.Thread(target=task1)
thread2 = threading.Thread(target=task2)

# Start threads
thread1.start()
thread2.start()
