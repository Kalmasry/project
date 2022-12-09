# Computer Eng Assessment 3
This is a sample readme file for our project, our project is to make a robot reach its goal safely without passing through the obstacles. To make that we used RRT path plan, we created circular obstacles with random sizes and assigned some functions/methods in the RRT class to create nodes until it finds a free space(without obstacles) to make a straight line to the goal. We used matplotlib,random,math libraries for path planning then we added a robot using roboticstoolbox and made it move on the nodes+lines the path plan makes.

## Table of contents:
* [Team Members](#team-members)
* [Python version and Libraries we used](#python-version-and-libraries-we-used)
* [Path planning using RRT](#path-planning-using-rrt)
* [Running RRT path plan flow chart](#running-rrt-path-plan-flow-chart)
* [Add a robot and make it move on the RRT path plan](#add-a-robot-and-make-it-move-on-the-rrt-path-plan)
* [Controlling robot movement flow chart](#controlling-robot-movement-flow-chart)
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

## Path planning using RRT:
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
## Running RRT path plan flow chart:
![image](https://user-images.githubusercontent.com/114658809/206564852-06696816-fb8f-47aa-b7ed-99b55b2b237e.png)

## visualization of RRT* Path Finding :
![image](https://user-images.githubusercontent.com/114488000/206625879-d2263cfe-4ebc-41fe-84c6-b2b5f4f93217.png)

## Add a robot and make it move on the RRT path plan:
```
# call and generate RRT
RT = RRT()

# list of plotting colors
# [start, end, points, path]
COLORS = ['#6AB71F', '#FF5733', '#4DAAEA', '#C0120A']

anim = VehicleIcon ('tire.png',scale=4)
initial_pos=[RRT.start[0],RRT.start[1]]
goal=[RRT.end[0],RRT.end[1]]

print(RRT.start[0],RRT.start[1])
print(RRT.end[0],RRT.end[1])

veh = Bicycle(
    animation=anim,
    control=RandomPath,
    dim=100, #the dimensions of the grid
    x0=(initial_pos[0],initial_pos[1],0),#inital position
)
veh.init(plot=True)

axes=plt.gca()#to put the robot figure and the rrt path planning figure in one figure
  
axes.set_xlim([-1,100])
axes.set_ylim([-1,100])

plt.scatter(RRT.end[0], RRT.end[1], s=200, c=COLORS[1], marker='2')#plot two-variables (rrt endx,rrtendy) in a point
#Creating cirular obstacles and adding them to the plot
OBSTACLES = [plt.Circle(j[0], j[1]) for i, j in enumerate(RRT.obstacle_props)]
OBS_PATCHES = mpl.collections.PatchCollection(OBSTACLES, facecolors='black')
axes.add_collection(OBS_PATCHES)
for k in enumerate(RRT.nodes_list):#enumerating through the nodes list then creating node to node on the graph until it reaches the goal
    plt.scatter(k[1][1][0], k[1][1][1], s=10, c=COLORS[2])
    if k[0] > 0:
        node_seg = mpl.collections.LineCollection(RRT.segments_list[k[0]-1:k[0]], colors=COLORS[2])
        axes.add_collection(node_seg)
    plt.pause(0.25)
plt.show()
for m in enumerate(RRT.path_nodes):#enumerating through the path nodes list then creating node to node on the nodes and plotting a final path when it finds  the goal
    plt.scatter(m[1][0], m[1][1], s=10, c=COLORS[3])
    if m[0] > 0:
        path_seg = mpl.collections.LineCollection(RRT.path_segments[m[0]-1:m[0]], colors=COLORS[3])
        axes.add_collection(path_seg)
    plt.pause(0.1)
plt.show()


x=[item[0] for item in RRT.path_nodes]
y=[item[1] for item in RRT.path_nodes]

for n in range (len(RRT.path_nodes)-1):
    run=True
    target=(x[n+1],y[n+1] )
    while (run):
        goal_heading=atan2(target[1]-veh.x[1],target[0]-veh.x[0])
        
        steer=goal_heading-veh.x[2]
        
        if steer>pi:
            steer = steer-2*pi
        veh.step(5,steer)
        
        if( (abs(target[0]-veh.x[0]) >0.3) or (abs(target[1]-veh.x[1]) >0.3) ):
            run=True
        else:
            run=False
        veh._animation.update(veh.x)
        plt.pause(0.00005)
plt.show()
```
## Controlling robot movement flow chart:

## Code Limitations:
Unforunately there are some limitations in this code and one of them is that RRT STAR path planning doesn't consider the robot size and if it can pass in the free space between the obstacles. Another limitation was the required model to use(car-like(bicycle in robotics toolbox)) it would've been better if we were allowed to use differential drive.   
<img width="311" alt="image" src="https://user-images.githubusercontent.com/114658809/206645643-2f2b125c-d7b8-4127-a8f0-566405f9b114.png">
