import random
import math
import matplotlib as mpl
import matplotlib.pyplot as plt

def calc_distance(p_1, p_2):#Calculates distance between two points, ARGUMENTS: point coordinates; tuple,  OUTPUT: Straight-line distance from p1->p2; scalar
    return math.sqrt((p_2[0]-p_1[0])**2 + (p_2[1]-p_1[1])**2)

def point_to_line(p_1, p_2, p_3):#Defines a line passing through two points. Determines whether the tangent to the line passing through a third point intersects between
     #the first two. OUTPUT: r_u p_1 -> p_2 distance intersection ratio; scalartan_len Distance from p_1 -> p_2 line to p_3; scalar
    
    dist = math.sqrt((p_2[0] - p_1[0])**2 + (p_2[1] - p_1[1])**2)# distance from P1 -> P2

    # determine intersection ratio u for three points A, B with a line between them and a third point C, the tangent to the line AB
    # passing through C intersects the line AB a distance along its length equal to u*|AB|
    r_u = ((p_3[0] - p_1[0])*(p_2[0] - p_1[0]) + (p_3[1] - p_1[1])*(p_2[1] - p_1[1]))/(dist**2)

    # intersection point
    p_i = (p_1[0] + r_u*(p_2[0] - p_1[0]), p_1[1] + r_u*(p_2[1] - p_1[1]))

    # distance from P3 to intersection point
    tan_len = calc_distance(p_i, p_3)

    return r_u, tan_len

class RRT(object):

    def __init__(self):

        self.side_len = 100

        self.num_obs = random.randint(15, 25)# number of obstacles
        self.obstacle_props = self.gen_obstacles(self.num_obs)# generate obstacle list

        self.start, self.end = self.gen_start_end()# start and end points

        self.nodes_list = [['q0', self.start, 'None']] # list of nodes in tree  [Node name, node coordinates, parent node]
        
                                                                    
        self.segments_list = []# list of node-to-node path segments

        self.gen_tree()

        self.path_nodes, self.path_segments = [], []# nodes/segments along the path from start to finish
                                                                

        self.find_path()                                                

    def gen_obstacles(self, num):#Generates a given number of circular objects, ARGUMENTS:num = Number of obstacles to generate; integer
        obstacles = []

        while len(obstacles) < num:
            overlap = []

            center = (self.side_len*random.random(), self.side_len*random.random())
            radius = 10*random.random()

            # iterate over obstacle list to check for collisions
            for _, props in enumerate(obstacles):
                if calc_distance(center, props[0]) >= radius + props[1]:
                    overlap.append(False)
                else:
                    overlap.append(True)

            if any(overlap):
                pass
            else:
                obstacles.append([center, radius])

        return obstacles

    def obstacle_check(self, point):#Checks whether a point is inside any of the obstacles. ARGUMENTS: point = Point to check tuple, OUTPUT: collision = Collision 
                                                                                                                      #condition; boolean (true if collision exists)
     
        for _, props in enumerate(self.obstacle_props):
            if calc_distance(point, props[0]) <= props[1]:
                return True
            else:
                pass

        return False

    def gen_start_end(self):#Generates start/end nodes in the RRT space and checks for collisions with obstacles. OUTPUT: start: Start point tuple, end: End point/goal tuple

        start_ok, end_ok = False, False

        while not start_ok:
            start = (10 + 20*random.random(), 10 + 20*random.random())

            if self.obstacle_check(start):
                pass
            else:
                start_ok = True

        while not end_ok:
            end = (70 + 20*random.random(), 70 + 20*random.random())

            if self.obstacle_check(end):
                pass
            else:
                end_ok = True

        return start, end

    def find_closest(self, point):#Finds the closest existing tree node to a given point. ARGUMENTS: point=Point to find closest node to, OUTPUT: d_list= List index of closest node in tree

        d_list = [calc_distance(point, node[1]) for node in self.nodes_list]

        return min(range(len(d_list)), key=d_list.__getitem__)

    def gen_node(self):#Generates nodes until it finds the goal
    
        point_ok = False
        node_name = "q{}".format(len(self.nodes_list))

        while not point_ok:
            # generate random coordinates
            p_coords = (self.side_len*random.random(), self.side_len*random.random())

            # find parent node
            parent = self.nodes_list[self.find_closest(p_coords)]

            # print(parent)

            # x- and y-distances to random point from parent node
            d_x = p_coords[0] - parent[1][0]
            d_y = p_coords[1] - parent[1][1]

            # magnitude of vector to closest node
            vec_mag = math.sqrt((d_x**2) + (d_y**2))

            # get new node coordinates by adding unit vector components to parent coordinates
            node = (parent[1][0] + d_x/vec_mag,
                    parent[1][1] + d_y/vec_mag)

            # if newly created node
            if self.obstacle_check(node):
                pass
            else:
                point_ok = True

        self.nodes_list.append([node_name, node, parent[0]])
        self.segments_list.append([parent[1], node])

    def path_check(self, point):#Checks for a clear straight-line path from a node to the end point.ARGUMENTS: point= point to check tuple, OUTPUT: Collision condition;
                                                                                                                                  # boolean (true if collision(s) present)

        # empty list to hold collision conditions between path and individual obstacles
        path_collisions = []

        # check for collision with each obstacle
        for obs in self.obstacle_props:
            too_close, between = False, False

            # return tangent distance and intersection ratio between obstacle center and path to end
            r_u, d_obs = point_to_line(point, self.end, obs[0])

            # determine if line segment and tangent through obstacle center intersect within segment bounds
            if 0 <= r_u <= 1:
                between = True

            # determine if intersection distance is smaller than obstacle radius
            if d_obs <= obs[1]:
                too_close = True

            # path is blocked if intersection is within segment bounds and it is is closer to obstacle center than obstacle radius length
            if between and too_close:
                path_collisions.append(True)
            else:
                path_collisions.append(False)

        return any(path_collisions)

    def gen_end_seg(self):#Generates final path segment and adds to list of segments.

        self.segments_list.append([self.nodes_list[-1][1], self.end])

    def gen_tree(self):#Generates a tree of nodes until it finds the goal to draw a straight line to the goal
        done = False

        while not done:
            self.gen_node()

            if not self.path_check(self.nodes_list[-1][1]):
                done = True

        self.gen_end_seg()

        self.nodes_list.append(["q{}".format(len(self.nodes_list)), self.end, self.nodes_list[-1][0]])

    def find_path(self):# Works backward through the list of points to find the path from start to finish.
        current = self.nodes_list[-1]# set end as current node
        self.path_nodes.append(current[1])# append end coordinates to list of path nodes

        for _, j in reversed(list(enumerate(self.nodes_list))):
            if current[2] == j[0]:
                self.path_nodes.insert(0, j[1])
                self.path_segments.insert(0, (j[1], current[1]))
                current = j
