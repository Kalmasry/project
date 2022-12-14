# Computer Eng Assessment 3
This is a sample readme file for our project, our project is to make a robot reach its goal safely without passing through the obstacles. To make that we used RRT path plan, we created circular obstacles with random sizes and assigned some functions/methods in the RRT class to create nodes until it finds a free space(without obstacles) to make a straight line to the goal. We used matplotlib,random,math libraries for path planning then we added a robot using roboticstoolbox and made it move on the nodes+lines the path plan makes.

## Table of contents:
* [Team Members](#team-members)
* [Python version and Libraries we used](#python-version-and-libraries-we-used)
* [Path planning using RRTSTAR](#path-planning-using-rrtstar)
* [RRTSTAR Robot Navigation](#rrtstar-robot-navigation)
* [Code Limitations](#code-limitations)

## Team Members:
* Abdelrahaman Mohamed Ghonamy
* Khalid Ahmed Almasry

## Python version and Libraries we used:
* python version 3.6.9
* random
* from math: atan2,pi
* matplotlib version 3.3.4 
* from roboticstoolbox(version 0.10.1): Bicycle, RandomPath, VehicleIcon

## Path planning using RRTSTAR:
```
* function calc_distance:
Calculates distance between two points

* function point_to_line:
Defines a line passing through two points, it determines whether the tangent to the line passing through a third point intersects between the first two.
```
### Class RRT:
```
* function init:
contains variables that are used once

* function gen_obstacles:
Generates a given number of circular objects, num = Number of obstacles to generate; integer

* function obstacle_check: 
Checks whether a point is inside any of the obstacles. point = Point to check tuple

* function gen_start_end:
Generates start/end nodes in the RRT space and checks for collisions with obstacles

* function find_closest:
Finds the closest existing tree node to a given point. point=Point to find closest node to

* function gen_node:
Generates nodes until it finds the goal

* function path_check:
Checks for a clear straight-line path from a node to the end point. point= point to check tuple

* function gen_end_seg: 
Generates final path segment and adds to list of segments.

* function gen_tree:
Generates a tree of nodes until it finds the goal to draw a straight line to the goal

* function find_path:
Works backward through the list of points to find the path from start to finish.
```
### Visualization of RRT* Path Finding :
![image](https://user-images.githubusercontent.com/114488000/206672837-721498a7-f4e8-4ac8-a93e-75514b28df64.png)

## RRTSTAR Robot Navigation:
![image](https://user-images.githubusercontent.com/114488000/206672507-b934d52b-8eaf-409f-b91f-3f148f6ca0c2.png)
![image](https://user-images.githubusercontent.com/114488000/206672984-0bcb9327-531a-4084-8e58-22694f650bc5.png)

### Controlling robot movement flow chart:
![image](https://user-images.githubusercontent.com/114658809/206681734-0fd934cb-a860-4671-a006-203dcf3d7d23.png)


## Code Limitations:
Unforunately there are some limitations in this code and one of them is that RRT STAR path planning doesn't consider the robot size and if it can pass in the free space between the obstacles. Another limitation was the required model to use(car-like(bicycle in robotics toolbox)) it would've been better if we were allowed to use differential drive, we needed a more advanced library for our project. We know that the movement of the robot is not the best and that it needs further progress in the future

![image](https://user-images.githubusercontent.com/114658809/206684422-622023d4-d9dc-4716-a643-ab4ed49af4f9.png)
