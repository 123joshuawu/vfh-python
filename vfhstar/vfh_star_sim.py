from vfh_star import *
import sys
import math
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


""" Testing for  algorithm using map.txt grid from vfh-python""" 
# USER-DEFINED VARIABLES--------

# Start/end location for robot in histogram grid
start = (47, 47)
end = (3, 3)

# Waypoints for robot path
waypoints = []#[(x, 46) for x in range(3, 45, 3)]
# waypoints = [(46, 46), (43, 30), (25, 15), (3, 3)]
# start = waypoints[0]
# end = waypoints[-1]

# Dimension of HistogramGrid
hg_dim = (50, 50)

# Number of sectors in PolarHistogram
nsectors = 72

# Window Length of active region
w_s = 21

# Radius of robot in terms of cells
rr = 1

# Safety distance between robot and obstacle in terms of cells
ds = 1

# Max number of steps for loop (Prevent infinite loops)
MAX_STEPS = 100

#CONSTANTS USED IN CALCS-----------------------------
#Feel free to change these to see what happens

# Positive constants for calculating cell magnitude
# Should satisty mhp_a - mhp_b * sqrt(2) * (w_s - 1)/2 = 0
cm_a = 29
cm_b = 2

# Positive constant for smoothing polar histogram
# mhp_l = 5

# Positive constants for certainty threshold of polar sector
t_low = 24
t_high = 35
# t_low = 40
# t_high = 50

# Positive constant for number of consecutive sectors for a wide valley
# Should change in accordance with nsectors
smax = 16

# Positive constants for calculating cost of candidate angles
# gbd_a is for goal oriented steering, gbd_b and gbd_c are for smooth steering
# Should satisty gbd_a > gbd_b + gbd_c
a = 5
b = 2
c = 2


# Positive constants for A star search 
# Projected step distance
d_s = math.sqrt(2)
# Total projected distance
d_t = d_s * 1

# Positive constants for projected costs and heuristics
mu1 = 5
mu2 = 1
mu3 = 1

# Positive constant between 0 and 1 for projected cost and heuristics
l = 0.8
#---------------

debug = True

def from_map(map_fname):
    """ Create grid from text file """
    with open(map_fname, 'r') as f:
        reader = csv.reader(f, delimiter=" ")
        lines = list(reader)

    lines = list(map(lambda l: list(map(int, l)), lines))
    nrows = len(lines)
    ncols = len(lines[0])

    return lines


current = start
target_angle = wrap_angle(math.degrees(math.atan2(end[1] - start[1], end[0] - start[0])))
current_angle = target_angle
previous_angle = target_angle

hg_poly = [(0,0), (0,49), (49, 49), (49, 33), (37, 33), (37, 20), (49, 20), (49, 0), (0, 0)]
# hg_poly = [(0,0), (0,49), (49, 49), (49, 0), (0, 0)]
hg = HistogramGrid(hg_dim[0], hg_dim[1], hg_poly)
# hg.histogram_grid = from_map("/Users/joshuawu/vfh-python/map_no_sides.txt")
hg.histogram_grid = from_map("map.txt")
trace_bounds(hg, hg_poly)

# coords = [(37.42302622166224,-122.11156738954924),(37.42298095636113,-122.11161231655024),(37.42294687423401,-122.11155800181768),(37.422991074489644,-122.11151709813021)]
# test = Zone("test", coords, 0.25)
# hg = test.hg
# hg_dim = hg.dimensions
# hg_poly = hg.poly.exterior.coords


ph = None

steps = []
index = 0


ani = None
fig = plt.figure()
ax = fig.add_subplot(111)
plt.ylim(0, len(hg.histogram_grid))
plt.xlim(0, len(hg.histogram_grid[0]))
ax.set_xticks(np.arange(0, len(hg.histogram_grid[0]), 1))
ax.set_yticks(np.arange(0, len(hg.histogram_grid), 1))
ax.set_axisbelow(True)
plt.grid()

plt.gca().axes.get_xaxis().set_ticklabels([])
plt.gca().axes.get_yaxis().set_ticklabels([])
# plt.gca().invert_yaxis()

for y in range(len(hg.histogram_grid)):
    for x in range(len(hg.histogram_grid[0])):
        if hg.histogram_grid[y][x] > 0:
            ax.plot(x + 0.5, len(hg.histogram_grid) - y - 1 + 0.5, 'ks')

for pt in waypoints:
    ax.plot(pt[0] + 0.5, pt[1] + 0.5, 'rx')
# ax.plot(end[0], end[1], 'rx')
pts = ax.scatter([start[0] + 0.5], [start[1] + 0.5])


from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
poly = Polygon(hg_poly)
p = PatchCollection([poly], alpha=0.4)
ax.add_collection(p)


steps_taken = None
estimated_arrival = None
dis_from_prev_goal = None
dis_to_next_goal = None
prev_goal = None
window_size = None

def pathplanning():
    global waypoints, steps_taken, estimated_arrival, dis_from_prev_goal, dis_to_next_goal, prev_goal, window_size

    # if len(waypoints) == 0 or index > MAX_STEPS:
    #     return None

    # goal = waypoints[0]
    # print("Goal", goal)

    # dis_from_prev_goal.append(math.hypot(current[1] - prev_goal[1], current[0] - prev_goal[0]))
    # dis_to_next_goal.append(math.hypot(current[1] - goal[1], current[0] - goal[0]))

    # if current == goal or steps_taken >= estimated_arrival:
    #     goal = waypoints[1] if len(waypoints) > 1 else waypoints[0]
    #     prev_goal = waypoints.pop(0)
    #     steps_taken = 0
    #     detour_constant = 0.1
    #     estimated_arrival = int(math.hypot(current[1] - goal[1], current[0] - goal[0]) ** 2 / d_s) * detour_constant
    #     # window_size = int(math.hypot(current[1] - goal[1], current[0] - goal[0]) / d_s)
    #     return pathplanning()


    # steps_taken += 1

    return a_star(hg, current, current_angle, end, d_t, d_s, w_s, nsectors, rr, ds, t_low, t_high, smax, cm_a, cm_b, a, b, c, mu1, mu2, mu3, l, steps)



def update(frame):
    global current, previous_angle, current_angle, index, steps 

    if current != end and index < MAX_STEPS:
        print("STEP", index)
        print("current", current)

        # try:

        best_angle = pathplanning()

        if best_angle is None:
            ani.event_source.stop()

        # best_angle = a_star(hg, current, current_angle, end, d_t, d_s, w_s, nsectors, rr, ds, t_low, t_high, smax, cm_a, cm_b, a, b, c, mu1, mu2, mu3, l, steps)

        # except Exception as e:
        #     print(e)
        #     ani.event_source.stop()

        print("best_angle", best_angle)
        steps.append((index, current, target_angle, wrap_angle(best_angle)))    
        
        # Compute next adjacent cell robot will be in 
        next_angle = math.floor(wrap_angle(best_angle + 22.5) / 45) * 45
        print("current", current)
        print("next_angle", next_angle)
        next_x = int(round((math.sqrt(2) if next_angle % 90 != 0 else 1) * math.cos(math.radians(next_angle))) + current[0])
        next_y = int(round((math.sqrt(2) if next_angle % 90 != 0 else 1) * math.sin(math.radians(next_angle))) + current[1])
        print("next x %d y %d" % (next_x, next_y))

        current = (next_x, next_y)
        if hg.out_of_bounds(next_x, next_y): raise Exception("WHY ARE WE GOING OUT???")
        previous_angle = current_angle
        current_angle = best_angle

        index += 1
        print("-" * 16)

        ofs = pts.get_offsets()
        ofs = np.append(ofs, [[current[0] + 0.5, current[1] + 0.5]], axis=0)
        pts.set_offsets(ofs)  
    else:
        ani.event_source.stop()  

    print("COMPLETE")


try:
    ani = animation.FuncAnimation(fig, update)
except Exception as e:
    print(e)
    ani.event_source.stop()
finally:
    for s in steps:
        print("{0:2}. ({1:2}, {2:<2}) target_angle: {3:5.1f}   best_angle: {4:5.1f}  ".format(s[0], s[1][0], s[1][1],  s[2], s[3]))

plt.show()


