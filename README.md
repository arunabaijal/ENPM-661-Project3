# ENPM-661-Project3

This repository contains the implementation of the A* algorithm on a turtlebot in gazebo.

## Dependencies
```
python3.5
Ubuntu 16.04
OpenCV 3.4.9
numpy
ROS Kinetic
turtlebot_gazebo
Turtlebot 2
```
## Running the code
To clone the repository, run git clone https://github.com/arunabaijal/ENPM-661-Project3.git
Check out branch phase4
Follow the following steps to successfully run the code:
```
1. Clone the repository into your catkin workspace src folder as project3
2. Navigate to folder project3
3. Run python3 Astar_rigid.py
4. Change absolute path to location of nodePath.txt in astar_turtlebot.py at line 54
5. After successfully finding path and generation of file nodePath.txt in project3 run catkin_make in your catkin_ws
6. Run roslaunch project3 project3.launch x_start:=4 y_start:=3 Y_start:=3.14 where x_start, y_start, z_start, R_start, P_start and Y_start are the initial pose parameters. (By default the turtlebot is spawned at x_start:=4 y_start:=3 Y_start:=3.14. Note that, for a theta value of 180, the turtlebot should be spawned at yaw value 3.14)
```
Upon running the code you would be asked to enter some details. See below for the sample input screenshot:
$ python3 Astar_rigid.py 
Please enter robot clearance value: 0.2
Please enter the x coordinate of the start point: 4
Please enter the y coordinate of the start point: 3
Please enter the theta value of the robot (in deg): 180
Please enter the x coordinate of the goal point: 0
Please enter the y coordinate of the goal point: 3
Please enter value of RPM1: 10
Please enter value of RPM2: 20


## Sample Output

Upon successfully finding the path the program returns the running time for finding the path and writes the path in file nodePath.txt. This file is read to publish velocity values to the turtlebot.


## Notes
For our turtlebot we have used the diameter of the robot (and distance between the wheels) as 354 mm, the radius of the wheels as 38 mm. We have assumed the center of the map to be the center of the world.
