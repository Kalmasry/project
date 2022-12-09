
import matplotlib as mpl
import matplotlib.pyplot as plt
from RRT import RRT

COLORS = ['#6AB71F', '#FF5733', '#4DAAEA', '#C0120A']

# call and generate RRT
RRT = RRT()



# create plot image
FIG, AX = plt.subplots(nrows=1, ncols=1, sharex=True, sharey=True, figsize=(9, 9))
# axis limits
plt.xlim(0, RRT.side_len)
plt.ylim(0, RRT.side_len)

# plot obstacles as circular patch collection
OBSTACLES = [plt.Circle(j[0], j[1]) for i, j in enumerate(RRT.obstacle_props)]
OBS_PATCHES = mpl.collections.PatchCollection(OBSTACLES, facecolors='black')
AX.add_collection(OBS_PATCHES)

# plot start and end points
plt.scatter(RRT.start[0], RRT.start[1], s=200, c=COLORS[0], marker='1')
plt.scatter(RRT.end[0], RRT.end[1], s=200, c=COLORS[1], marker='2')

# plot all nodes/edges one by one
for k in enumerate(RRT.nodes_list):
    plt.scatter(k[1][1][0], k[1][1][1], s=10, c=COLORS[2])
    if k[0] > 0:
        node_seg = mpl.collections.LineCollection(RRT.segments_list[k[0]-1:k[0]], colors=COLORS[2])
        AX.add_collection(node_seg)
    plt.pause(0.001)

# plot path nodes/edges one by one
for m in enumerate(RRT.path_nodes):
    plt.scatter(m[1][0], m[1][1], s=10, c=COLORS[3])
    if m[0] > 0:
        path_seg = mpl.collections.LineCollection(RRT.path_segments[m[0]-1:m[0]], colors=COLORS[3])
        AX.add_collection(path_seg)
    plt.pause(0.006)

plt.show()
