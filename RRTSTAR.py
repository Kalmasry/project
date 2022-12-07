import random 
import math
import pygame
import time

#class methods: drawing the map,start and goal points,store obstacles temporarily, and draw the path calculated
class RRTMAP:
    def _init_(self,start,goal,MapDimensions,obsdim,obsnum):
        #Declare the variables
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.Maph,self.Mapw=MapDimensions #splitting it into 2 variables width and height
    
        #Name our window
        self.MapwindowName='RRT path planning'
        pygame.display.set_caption(self.MapwindowName)#we used set caption from pygame to name the window
        self.map=pygame.display.set_mode((self.Mapw,self.Maph))#we used set mode from pygame to create canvas we will work on
        self.map.fill((255,255,255))#filling the canvas with white color to make the background
        self.nodeRad=2
        self.nodeThickness=1
        self.edgeThickness=1

        self.obstacles=[]#storing obstacles in this list
        self.obsdim = obsdim
        self.obsNumber = obsnum

        #declaring colors using their rgb codes
        self.grey = (70,70,70)
        self.blue = (0,0,255)
        self.green = (0,255,0)
        self.red = (255,0,0)
        self.white = (255,255,255)

    #draws start and goal as circles
    def drawmap(self,obstacles):
        pygame.draw.circle(self.map,self.green,self.start,self.nodeRad+5,0)#draws start and goal as circles (noderad+5 is the circle size) (0 is full circle) (1 is hollow circle)
        pygame.draw.circle(self.map,self.green,self.goal,self.nodeRad+20,1)
        self.drawobs(obstacles)


    #takes obstacles as input and copies them in a temporary variable
    def drawobs(self,obstacles):
        obstacleslist=obstacles.copy()#takes the obstacles as input and copies them in a temporary variable
        while (len(obstacleslist)>0):#we made a while loop so we keep popping from the list until it is empty
            obstacle = obstacleslist.pop(0)#removes the value at index 0 from the list
            pygame.draw.rect(self.map,self.grey,obstacle)# we use pygame.draw.rect to draw the obstacles on the screen (self.map is surface to draw on) (color of the retangles are set to be grey)

    def drawPath(self, path):#draw the path
        for node in path:
            pygame.draw.circle(self.map, self.red, node, 3, 0)




#class methods:making obstacles, adding and removing nodes and edges, checking collisions,find nearest points
class RRTGRAPH:
    def _init_(self,start,goal,MapDimensions,obsdim,obsnum):
        #we declared the same arguments that are in rrtmap
        (x,y)=start #start coordinates are seperated into x and y
        self.start = start
        self.goal = goal
        self.goalFlag = False #will be set to true if the tree reaches the goal region 
        self.maph,self.mapw=MapDimensions
        self.x=[]#empty list created for storing x coordinates of every node
        self.y=[]#empty list created for storing y coordinates of every node
        self.parent=[]#empty list created for storing the parent of those nodes
        #initialize the tree
        self.x.append(x)#we will append x and y to their lists:
        self.y.append(y)
        self.parent.append(0)#parent of start node the start node itself so we append 0 as the first element
        #needed for drawing obstacles
        self.obstacles=[]
        self.obsDim = obsdim
        self.obsNumber = obsnum
        #path
        self.goalstate=None #set a flag to check if we reached the goal region or no
        self.path=[]#empty list to hold calculated path

    def makeRandomRect(self):#random coordinates will represent the upper left corner of the rectangle(obstacle)
        ucx=int(random.uniform(0,self.mapw-self.obsDim))#we subtract obsdim to make sure that the obstacles will not generate a part of it out of the map
        ucy=int(random.uniform(0,self.maph-self.obsDim))
            
        return(ucx,ucy)
            
        #creating obstacles and storing them in a list
    def makeobs(self):
        obs = []
        for i in range(0,self.obsNumber):
            rectang = None #holding the obstacles temporarily before it gets stored
            startgoalcol = True #flag to indicate our start and goal position is in the map inside a newly created obstacle
            while startgoalcol:
                upper = self.makeRandomRect()#create a rondom upper corner 
                rectang=pygame.Rect(upper,(self.obsDim,self.obsDim))#made it it a full rectangle using pygame.Rect
                if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):#if rectangle(obstacle) collides with the start or goal point we will set a true flag
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rectang)#rectangle is added to the obs list
        self.obstacles=obs.copy()#the obs list is copied to the class variable self.obstacles
        return(obs)

    def add_node(self,n,x,y):#n=identification number(id) of the node with its x and y coordinates
        self.x.insert(n,x)#insert x and y in the x and y lists
        self.y.insert(n,y)

    def remove_node(self,n):#give the id number and it will pop its x and y coordinates
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self,parent,child):
        self.parent.insert(child,parent)#insert the parent of the child in the parents list (child is used as an index, parent stored as an element)

    def remove_edge(self,n):
        self.parent.pop(n)#cuts relationship between parent and child nodes

    def num_of_nodes(self):#know latest node added to the tree and the exact number of nodes
        return len(self.x)#return length of the x list which is equal to the total number of nodes

    def distance(self,n1,n2):#to measure the distance between 2 nodes, give the ids of the 2 nodes
        (x1,y1)=(self.x[n1],self.y[n1])#extract the x and y coordinates from the x and y lists
        (x2,y2)=(self.x[n2],self.y[n2])
        px=(float(x1)-float(x2))**2 #calculate the euclidean distance between them(px=(x1-x2)**2,py=(y1-y2)**2,(px+py)**(0.5))
        py=(float(y1)-float(y2))**2
        return (px+py)**(0.5)

    def sample_envir(self):#generate random samples from the map using random.uniform
        x=int(random.uniform(0,self.mapw))#map borders is our limit to make sure it doesnt get out of the graph
        y=int(random.uniform(0,self.maph))
        return(x,y)

    def nearest(self, n):#takes new added node that was sampled from the environment and measure the distance to every node in the tree
        dmin=self.distance(0,n)#dmin is the starting point which is the measured distance from 0 to n 
        nnear=0 # nnear is to hold the id of the closest node that founded
        for i in range (0,n):
            if self.distance (i,n)<dmin: # if the distance between the node i and the our node is less than dmin
                dmin=self.distance(i,n) # it will be assigned as a new value of dmin 
                nnear=i # and also assign the node id to near
        # this loop will continue till the distance to all nodes are measured and the nearest node is extracted 
        return(nnear)

    def isfree(self):#to check if the added node is located in a free space or no
        n =self.num_of_nodes()-1#to get id of the newly added node we subtract 1 from the total num of nodes because id of the nodes start with zero 
        (x,y)=(self.x[n],self.y[n])#extract x and y coordinates from their lists
        obs=self.obstacles.copy()#the class variable self.obstacles list is copied to a temporary list
        while len(obs)>0:#we made a while loop so we keep popping from the list until it is empty
            rectang=obs.pop(0)#removes the value at index 0 from the list
            if rectang.collidepoint(x,y):#if rectangle collides with the node we will remove the node by calling the function/method we created
                self.remove_node(n)
                return False#returning false=the node isnt free
        return True

    def crossobs(self,x1,x2,y1,y2): #checks if an edge(connection between 2 nodes) crosses any obstacles
        obs=self.obstacles.copy()#the class variable self.obstacles list is copied to a temporary list
        while (len(obs) > 0):#we made a while loop so we keep popping from the list until it is empty
            rectang = obs.pop(0)#removes the value at index 0 from the list
            for i in range(0, 101):#for loop to create 100 intermediate checkpoints between the two nodes
                u = i/100#we did i/100 because it will generate 100 checkpoints
                x = x1*u + x2*(1 - u)#used this formula because we want to check the ith checkpoint when i reaches 100 u=1 which will be = to the coordinates of the node
                y = y1*u + y2*(1 - u)
                if rectang.collidepoint(x, y):#if the obstacle being checked collides with the ith checkpoint 
                    return True
        return False

    def step(self, nnear, nrand, dmax=35):#create a new node between the new node and the nearest node
       d=self.distance(nnear,nrand)# let we have a two points nnear and nrand and we need need to create a point in the middle 
       if d>dmax:
            u=dmax/d
            (xnear,ynear)=(self.x[nnear],self.y[nnear])# let we have a node at point a (xnear,ynear)
            (xrand,yrand)=(self.x[nrand],self.y[nrand])# let we have a node at point c (xrand,yrand)
            #create a node between 2 nodes given in a predefined step size
            (px,py)=(xrand-xnear,yrand-ynear)# the distance between the two point 
            theta=math.atan2(py.px) # the angle between them 
            #xnear+stepsize*costheta(math rule is x1=x+n*costheta)
            (x,y)=(int(xnear+dmax*math.cos(theta)),int(ynear+dmax*math.sin(theta))) # then we can easliy calculate the coordinate of the middle point by using silmilarty and pythagoras theorem
            self.remove_node(nrand)# then we have to remove the nrand point after we created a near one
            # we need now to check if the new cereated node is inside the goal region of maximum step 35  if true then we founf the goal and no need to add the calculated node to the tree 
            if(x-self.goal[0])<dmax and abs(y-self.goal[1])<dmax:
                self.add_node(nrand,self.goal[0],self.goal[1]) # if the condition is true we add the goal itself instead of nrand node 
                self.goalstate=nrand # the nrand node will be the goalstate point with a same id in the memory 
                self.goalflag=True 
            else:
                self.add_node(nrand,x,y) # the  nrand will added to the tree if the condition not fulfilled 
    
    def bias(self, ngoal):#expansion in the direction of the goal
        n=self.num_of_nodes()# generating a new index to use it in creating the new node
        self.add_node(n,ngoal[0],ngoal[1])# add node  will add the goal to the tree
        nnear=self.nearest(n) # we get the nearest node in the tree to the goal
        self.step(nnear,n) # we call step method to create a permanent node and add it to the tree between the nearest node and the goal 
        self.connectnodes(nnear,n) # to connect the newly node to the nnear node
        return (self.x,self.y,self.parent)  

    def expand(self):#random expansion of the tree
        n=self.num_of_nodes() # generating a new index to use it in creating a new node 
        x,y=self.sample_envir() # generating a random samples points
        self.add_node(n,x,y) # add the new node to the tree
        if self.isfree():# if the new node in the free space
            xnearest=self.nearest(n)# get the nearest node to it 
            self.step(xnearest,n)# take a step from x nearest to create a new node  
            self.connect(xnearest,n)# connect the node created above in step to the nearest node
            return self.x,self.y,self.parent

    def connectnodes(self,n1,n2):#connect 2 nodes
        (x1,y1)=(self.x[n1],self.y[n1])#extract x and y coordinates from their lists
        (x2,y2)=(self.x[n2],self.y[n2])
        if self.crossobs(x1,x2,y1,y2):#we call crossobs method to check if the is connection edge between them
            self.remove_node(n2)#if there is a collision we remove the new node and return false
            return False
        else:
            self.add_edge(n1,n2)#if not call add edge method and provide n1 as a parent
            return True
    
    def path_to_goal(self):#extracting nodes that will make the path from the goal node to the start node
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):#extract the path nodes whenever we neeed them
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
        return pathCoords

    def cost(self, n):
        ninit = 0
        n = n
        parent = self.parent[n]
        c = 0
        while n is not ninit:
            c = c + self.distance(n, parent)
            n = parent
            if n is not ninit:
                parent = self.parent[n]
        return c

    def getTrueObs(self, obs):
        TOBS = []
        for ob in obs:
            TOBS.append(ob.inflate(-50, -50))
        return TOBS

    def waypoints2path(self):
        oldpath = self.getPathCoords()
        path = []
        for i in range(0, len(self.path) - 1):
            print(i)
            if i >= len(self.path):
                break
            x1, y1 = oldpath[i]
            x2, y2 = oldpath[i + 1]
            print('---------')
            print((x1, y1), (x2, y2))
            for i in range(0, 5):
                u = i / 5
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                path.append((x, y))
                print((x, y))

        return path
