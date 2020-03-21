# ENPM-661-Project3

This repository contains the implementation of the A* algorithm on a map.

## Dependencies
```
python3.5
Ubuntu 16.04
OpenCV 3.4.9
numpy
```
## Running the code

Follow the following steps to successfully run the code:
```
1. Go to the repository where the code is present on the terminal
2. python3 Astar_rigid.py
```
Upon running the code you would be asked to enter some details. See below for the sample input screenshot:
![Sample Input][Input_demo.png]

## Sample Output

Upon successfully finding the path the program returns the running time for finding the path and generating the video separately. The generated video is strored in the parent directory by default. For the sample input with start point:[50,30,60], end point:[150,150], radius=1 and clearance=1, the running time statistics and the path found screenshot are shared below:
```
Time taken to find path: 114.60394072532654 sec
Time taken to animate paths: 63.39507246017456 sec
```
![Path Found][path_found.png]