from vfh_plus import *
import sys
import math
import csv
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.animation as animation


# Need to
# - change movement of robot on actual map to decimal accuracy


""" Testing for VFH+ algorithm using map.txt grid from vfh-python"""
# USER-DEFINED VARIABLES--------

# Start/end location for robot in histogram grid
start = (3, 47)
end = (3, 3)

# Dimension of HistogramGrid
hg_dim = (50, 50)

# Number of sectors in PolarHistogram
nsectors = 72

# Window Length of active region
w_s = 21

# Max number of steps for loop (Prevent infinite loops)
MAX_STEPS = 100

# Robot velocity (unit square/interval)
v_r = 1

# CONSTANTS USED IN CALCS-----------------------------
# Feel free to change these to see what happens

# Positive constants for calculating cell magnitude
# Should satisty mhp_a - mhp_b * sqrt(2) * (w_s - 1)/2 = 0
mhp_a = 28
mhp_b = 2

# Positive constant for smoothing polar histogram
mhp_l = 5

# Positive constants for certainty threshold of polar sector
gbd_t_low = 22
gbd_t_high = 24

# Positive constant for number of consecutive sectors for a wide valley
# Should change in accordance with nsectors
gbd_smax = 18

# Positive constants for calculating cost of candidate angles
# gbd_a is for goal oriented steering, gbd_b and gbd_c are for smooth steering
# Should satisty gbd_a > gbd_b + gbd_c
gbd_a = 5
gbd_b = 2
gbd_c = 2
# ---------------


def from_map(map_fname):
    """ Create grid from text file """
    with open(map_fname, 'r') as f:
        reader = csv.reader(f, delimiter=" ")
        lines = list(reader)

    lines = list(map(lambda l: list(map(int, l)), lines))
    nrows = len(lines)
    ncols = len(lines[0])

    return lines


current_loc = start
current_cell = start
target_angle = None
current_angle = wrap_angle(-1 *
                           math.degrees(math.atan2(end[1] - start[1], end[0] - start[0])))
previous_angle = current_angle

hg = HistogramGrid(hg_dim[0], hg_dim[1])
# hg.grid = from_map("vfh-python/map_no_sides.txt")
hg.grid = from_map("map.txt")

ph = None

steps = []
index = 0

ani = None

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xticks(np.arange(0, len(hg.grid[0]), 5))
ax.set_yticks(np.arange(0, len(hg.grid), 5))
plt.grid()
# plt.gca().invert_yaxis()

for y in xrange(len(hg.grid)):
    for x in xrange(len(hg.grid)):
        if hg.grid[y][x] == 1:
            ax.plot(x, len(hg.grid) - y - 1, 'ks')
ax.plot(end[0], end[1], 'rx')
pts = ax.scatter([start[0]], [start[1]])

# def test():
#     ph = VFH.map_active_hg_to_ph(hg, PolarHistogram(nsectors), current, w_s, mhp_a, mhp_b, mhp_l)
#     target_angle = wrap_angle(-1 * math.degrees(math.atan2(end[1] - current[1], end[0] - current[0])))
#     best_angle = VFH.get_best_direction(ph, target_angle, current_angle, previous_angle, gbd_t_low, gbd_t_high, gbd_smax, gbd_a, gbd_b, gbd_c)


def update(frame):
    global index, current_cell, current_loc, current_angle, target_angle, previous_angle, steps, v_r

    if hg.out_of_bounds(current_cell[0], current_cell[1]):
        ani.event_source.stop()
        return

    if current_cell == end:
        print("Goal reached")
        ani.event_source.stop()
        return
    # try:
    if index < MAX_STEPS:
        print "STEP", index
        #hg.print_hg(list(map(lambda s: s[1], steps)), start, end, current)

        ph = VFH.map_active_hg_to_ph(hg, PolarHistogram(
            nsectors), current_cell, w_s, mhp_a, mhp_b, mhp_l)
        # avg = np.median(ph.polar_histogram)#sum(ph.polar_histogram) / len(ph.polar_histogram)

        target_angle = wrap_angle(math.degrees(math.atan2(
            end[1] - current_loc[1], end[0] - current_loc[0])))
        best_angle = VFH.get_best_direction(
            ph, target_angle, current_angle, previous_angle, gbd_t_low, gbd_t_high, gbd_smax, gbd_a, gbd_b, gbd_c)
        print "current_cell (%d, %d)" % (current_cell[0], current_cell[1])
        print "current_loc (%f, %f)" % (current_loc[0], current_loc[1])
        print "best_angle", best_angle
        steps.append((index, current_cell, current_loc,
                      target_angle, wrap_angle(best_angle)))

        # check if robot needs to slow down
        end_dis = math.hypot(end[1] - current_loc[1], end[0] - current_loc[0])
        print "end_dis %d" % end_dis
        if end_dis < v_r:
            v_r = end_dis

        # Compute next adjacent cell robot will be in
        dx = v_r * math.cos(math.radians(best_angle))
        dy = v_r * math.sin(math.radians(best_angle))
        print "dx %f dy %f" % (dx, dy)
        next_x = int(math.floor(current_loc[0] + dx))
        next_y = int(math.floor(current_loc[1] + dy))
        print "next (%d, %d)" % (next_x, next_y)

        current_loc = (current_loc[0] + dx, current_loc[1] + dy)
        current_cell = (next_x, next_y)
        previous_angle = current_angle
        current_angle = best_angle

        index += 1
        print "-" * 16

        ofs = pts.get_offsets()
        ofs = np.append(ofs, [[current_loc[0], current_loc[1]]], axis=0)
        pts.set_offsets(ofs)
    # except Exception:
    #     ani.event_source.stop()

    # finally:
    #     for s in steps:
    #             print "{0:2}. ({1:2}, {2:<2}) -- ({3:2.1f}, {4:<2.1f}) target_angle: {5:5.1f}   best_angle: {6:5.1f} ".format(s[0], s[1][0], s[1][1], s[2][0], s[2][1], s[3], s[4])

    # print "COMPLETE"

    # return pts

        #hg.print_hg(list(map(lambda s: s[1], steps)), start, end, current)

#         #if raw_input(): break
# finally:
#     i = 0
#     for s in steps:
#         print "{0:2}. ({1:2}, {2:<2}) target_angle: {3:5.1f}   best_angle: {4:5.1f}   smoothed_ph cntr: {5:3.1f}   avg_obs_dis: {6}".format(s[0], s[1][0], s[1][1],  s[2], s[3], s[4], s[5])
#         if i < len(steps) - 1 and steps[i + 1][4] - s[4] > gbd_t_high and steps[i + 1][5] > s[5]:
#             print "WOWWWW", steps[i + 1][4] - s[4]
#         i += 1


#     hg.print_hg(list(map(lambda s: s[1], steps)), start, end, current)
try:
    ani = animation.FuncAnimation(fig, update)
except Exception:
    print "wowww"
    ani.event_source.stop()

plt.show()


# try:
#     print "VARS INITIALIZED STARTING LOOP"
#     while index < MAX_STEPS:
#         print "STEP", index
#         #hg.print_hg(list(map(lambda s: s[1], steps)), start, end, current)
#         if current == end: break

#         ph = VFH.map_active_hg_to_ph(hg, PolarHistogram(nsectors), current, w_s, mhp_a, mhp_b, mhp_l)
#         avg = np.median(ph.polar_histogram)#sum(ph.polar_histogram) / len(ph.polar_histogram)

#         target_angle = wrap_angle(-1 * math.degrees(math.atan2(end[1] - current[1], end[0] - current[0])))
#         best_angle = VFH.get_best_direction(ph, target_angle, current_angle, previous_angle, gbd_t_low, gbd_t_high, gbd_smax, gbd_a, gbd_b, gbd_c)
#         print "best_angle", best_angle
#         steps.append((index, current, target_angle, wrap_angle(best_angle), avg, ph.avg_obs_dis, ph.polar_histogram))

#         # Compute next adjacent cell robot will be in
#         next_angle = math.floor(wrap_angle(best_angle + 22.5) / 45) * 45
#         print "current", current
#         print "next_angle", next_angle
#         next_x = int(round((math.sqrt(2) if next_angle % 90 != 0 else 1) * math.cos(math.radians(next_angle))) + current[0])
#         next_y = int(round((math.sqrt(2) if next_angle % 90 != 0 else 1) * math.sin(math.radians(next_angle))) * -1 + current[1])
#         print "next x %d y %d" % (next_x, next_y)

#         current = (next_x, next_y)
#         previous_angle = current_angle
#         current_angle = best_angle

#         index += 1
#         print "-" * 16

#     print "COMPLETE"


# vcp = (int(sys.argv[2]), int(sys.argv[3]))
# print vcp
# hg_dim = (50, 50)
# nsectors = 36
# w_s = 21

# old_hg = old.HistogramGrid.from_map("map.txt", 1)
# hg = HistogramGrid(hg_dim[0], hg_dim[1])
# hg.grid = old_hg.grid
# #print hg
# ph = VFH.map_active_hg_to_ph(hg, PolarHistogram(nsectors), vcp, w_s)
# print ph
# print VFH.get_best_direction(ph.polar_histogram, int(sys.argv[1]), smax=5)
# #ph = PolarHistogram(hg, (17, 12), 5, 16)
# #print ph
# #print ph.get_best_angle(180, 180)
