import matplotlib as mpl
import matplotlib.pyplot as plt

from roboticstoolbox import Bicycle , RandomPath, VehicleIcon
from math import pi, atan2
from RRT import RRT
RRT=RRT() # genrate all RRT class

def main():
    
    COLORS = ['#6AB71F', '#FF5733', '#4DAAEA', '#C0120A']
    
    
    anim = VehicleIcon ('car.png',scale=5)
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
    axes=plt.gca() # to add the obstacles in the same figure of robot
    axes.set_xlim([-1,100])
    axes.set_ylim([-1,100])
    plt.scatter(RRT.start[0], RRT.start[1], s=200, c=COLORS[0], marker='1')
    plt.scatter(RRT.end[0], RRT.end[1], s=200, c=COLORS[1], marker='2')
    OBSTACLES = [plt.Circle(j[0], j[1]) for i, j in enumerate(RRT.obstacle_props)]
    OBS_PATCHES = mpl.collections.PatchCollection(OBSTACLES, facecolors='black')
    axes.add_collection(OBS_PATCHES)
    
    for k in enumerate(RRT.nodes_list):
        plt.scatter(k[1][1][0], k[1][1][1], s=10, c=COLORS[2])
        if k[0] > 0:
            node_seg = mpl.collections.LineCollection(RRT.segments_list[k[0]-1:k[0]], colors=COLORS[2])
            axes.add_collection(node_seg)
        #plt.pause(0.25)
    #plt.show()
    for m in enumerate(RRT.path_nodes):
        plt.scatter(m[1][0], m[1][1], s=10, c=COLORS[3])
        if m[0] > 0:
            path_seg = mpl.collections.LineCollection(RRT.path_segments[m[0]-1:m[0]], colors=COLORS[3])
            axes.add_collection(path_seg)
        #plt.pause(0.1)
    #plt.show()


    x=[item[0] for item in RRT.path_nodes]
    y=[item[1] for item in RRT.path_nodes]

    for n in range (len(RRT.path_nodes)-1):
        run=True
        target=[x[n+3],y[n+3]]
        while (run):
            goal_heading=atan2(
            target[1]-veh.x[1],
            target[0]-veh.x[0]
            )
        
            steer=goal_heading-veh.x[2]
        
            if steer>pi:
                steer = steer-2*pi
            veh.step(3,steer)
        
            if( (abs(target[0]-veh.x[0]) >0.3) or (abs(target[1]-veh.x[1]) >0.3) ):
                run=True
            else:
                run=False
            veh._animation.update(veh.x)
            plt.pause(0.000005)
            
    plt.show()
if __name__ == '__main__':
    main()
