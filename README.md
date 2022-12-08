# Computer Eng Assessment 3
This is a sample readme file for our project, our project is to make a robot reach its goal safely without passing through the obstaclesan. To make that we used RRT path plan, we created circular obstacles with random sizes and assigned some functions/methods in the RRT class to create nodes until it finds a free space(without obstacles) to make a straight line to the goal. We used matplotlib,random,math,matplotlib.pyplot libraries for path planning then we added a robot using roboticstoolbox and made it move on the nodes+lines the path plan makes.

## Table of contents:
* [Team Members](#team-members)
* [Python version and Libraries we used](#python-version-and-libraries-we-used)
* [Path planning using RRT](#path-planning-using-rrt)
* [Add a robot and make it move on the RRT path plan](#add-a-robot-and-make-it-move-on-the-rrt-path-plan)

## Team Members:
* Abdelrahaman Mohamed Ghonamy
* Khalid Ahmed Almasry

## Python version and Libraries we used:
* python version 3.6.9
* random
* math
* matplotlib version 3.3.4
* matplotlib.pyplot 
* from roboticstoolbox(version 0.10.1): Bicycle, RandomPath, VehicleIcon

## Path planning using RRT
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
## Add a robot and make it move on the RRT path plan:
