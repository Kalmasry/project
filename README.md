#Computer Eng Assessment 3
This is a sample readme file for our project, our project is to make a robot reach its goal safely without passing through the obstacles.

##Path planinng using RRT

*function calc_distance:
Calculates distance between two points

*function point_to_line(p_1, p_2, p_3):
Defines a line passing through two points, it determines whether the tangent to the line passing through a third point intersects between the first two.

###CLass RRT:

*function init:
contains variables that are used once

*function gen_obstacles:
Generates a given number of circular objects, num = Number of obstacles to generate; integer

*function obstacle_check: 
Checks whether a point is inside any of the obstacles. point = Point to check tuple

*function gen_start_end:
Generates start/end nodes in the RRT space and checks for collisions with obstacles

*function find_closest:
Finds the closest existing tree node to a given point. point=Point to find closest node to

*function gen_node:
Generates nodes until it finds the goal

*function path_check:
Checks for a clear straight-line path from a node to the end point. point= point to check tuple

*function gen_end_seg: 
Generates final path segment and adds to list of segments.

*function gen_tree:
Generates a tree of nodes until it finds the goal to draw a straight line to the goal

*function find_path:
Works backward through the list of points to find the path from start to finish.
