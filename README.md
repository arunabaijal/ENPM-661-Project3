# ENPM-661-Project3

This repository contains the implementation of the A* algorithm with a turtlebot.

## Dependencies
```
python3.5
Ubuntu 16.04
OpenCV 3.4.9
numpy
```
## Running the code
To clone the repository, run git clone https://github.com/arunabaijal/ENPM-661-Project3.git
Check out branch phase3
Follow the following steps to successfully run the code:
```
1. Go to the repository where the code is present on the terminal
2. python3 Astar_rigid.py
```
Upon running the code you would be asked to enter some details. See below for the sample input:

Please enter robot clearance value: 0.2
Please enter the start point in this format - [x,y,theta (in deg)]: [-4,-4,0]
Please enter the goal point in this format - [x,y]: [4,4]
Please enter value of RPM1: 10
Please enter value of RPM2: 20


## Sample Output

Upon successfully finding the path the program returns the running time for finding the path and generating the video separately. The generated video is strored in the parent directory by default. For the sample input with start point:[-4,-4,0], end point:[4,4], and clearance=0.2, the running time statistics is shared below:
```
Time taken to find path: 96.10569739341736
Time taken to animate paths: 553.5960557460785
```

## Notes
For our turtlebot we have used the diameter of the robot (and distance between the wheels) as 354 mm, the radius of the wheels as 38 mm. We have assumed the center of the map to be the center of the world.
