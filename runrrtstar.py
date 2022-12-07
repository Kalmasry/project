import random 
import math
import pygame
import time
from RRTSTAR import RRTGRAPH, RRTMAP

def main():
    dimensions =(600,1000)
    start=(50,50)
    goal=(250,250)
    obsdim=30
    obsnum=50
    iteration=0
    t1=0
    pygame.init()
    map=RRTMAP(start,goal,dimensions,obsdim,obsnum)
    graph=RRTGRAPH(start,goal,dimensions,obsdim,obsnum)

    obstacles=graph.makeobs()
    map.drawmap(obstacles)

    t1=time.time()
    while (not graph.path_to_goal()):
        time.sleep(0.005)
        elapsed=time.time()-t1
        t1=time.time()
        #faise exception if timeout
        if elapsed > 10:
            print('time out re-initiating the calculations')
            raise

        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goal)
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad*2, 0)
            pygame.draw.line(map.map, map.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             map.edgeThickness)

        else:
            X, Y, Parent = graph.expand()
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad*2, 0)
            pygame.draw.line(map.map, map.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             map.edgeThickness)

        if iteration % 5 == 0:
            pygame.display.update()
        iteration += 1
    map.drawPath(graph.getPathCoords())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(10)



if __name__ == '__main__':#if result=true, loop terminates and if result=false it will keep trying until the path is found
    result=False
    while not result:
        try:
            main()
            result=True
        except:
            result=False