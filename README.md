### Vision-Based Dynamic Warehouse Automation using Swarm Robotics

Inspired by Amazon's Pegasus robots used for warehouse management, this project aims to employ a swarm of robots to pick up various objects and place them at different destinations based on situational requirements. A single global camera provides feedback to localize the robots, objects, and destinations.

This GitHub repository is divided into two parts: the server section and the robot section. Below is an overview of the robot section:

#### Camera Node
A single global camera is used to localize the robots and objects that need to be moved. This is achieved using a camera node in ROS2. The pose of the robots is estimated using ArUco markers. The camera node divides the workspace into rows and columns, publishing data such as ArUco marker IDs, row numbers, and column numbers using custom ROS2 interfaces to various ROS2 nodes.

#### Assigner Node
The Assigner Node receives data from the Camera Node about the robots and objects, and assigns robot-object pairs. In scenarios with multiple objects and robots, this node prioritizes pairs based on proximity. The assignment data is then sent to the Path Planning Node for path generation.

#### Path Planning Node
The Path Planning Node receives data from both the Camera Node and the Assigner Node. It simulates the environment using the camera feed and generates a path through it using the A* motion planning algorithm. Using custom ROS2 interfaces, an array of waypoints is provided to the robots to execute the determined path.

#### Robot Node
The Robot Node receives the path from the Path Planning Node and the robot's current position from the Camera Node. The path data, given as x and y coordinates, is used to calculate the required orientation of the robot. A PID controller serves as the control algorithm, interfacing with the Raspberry Pi's GPIO pins to control the motors.

By leveraging these components, the project demonstrates a dynamic and efficient approach to warehouse automation using swarm robotics and vision-based localization.