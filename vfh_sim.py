from vfh_hopefully_2 import *
import sys
import math
import csv

""" Testing for VFH+ algorithm using map.txt grid from vfh-python""" 
# USER-DEFINED VARIABLES--------

# Start/end location for robot in histogram grid
start = (17, 0)
end = (17, 49)

# Dimension of HistogramGrid
hg_dim = (50, 50)

# Number of sectors in PolarHistogram
nsectors = 72

# Window Length of active region
w_s = 21

# Max number of steps for loop (Prevent infinite loops)
MAX_STEPS = 100

#CONSTANTS USED IN CALCS-----------------------------
#Feel free to change these to see what happens

# Positive constants for calculating cell magnitude
# Should satisty mhp_a - mhp_b * sqrt(2) * (w_s - 1)/2 = 0
mhp_a = 1
mhp_b = 1

# Positive constant for smoothing polar histogram
mhp_l = 5

# Positive constant for certainty threshold of polar sector
gbd_t = 20

# Positive constant for number of consecutive sectors for a wide valley
# Should change in accordance with nsectors
gbd_smax = 18

# Positive constants for calculating cost of candidate angles
# gbd_a is for goal oriented steering, gbd_b and gbd_c are for smooth steering
# Should satisty gbd_a > gbd_b + gbd_c
gbd_a = 5
gbd_b = 2
gbd_c = 2
#---------------



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
target_angle = None
current_angle = wrap_angle(math.degrees(math.atan2(start[0] - end[0], start[1] - end[1])))
previous_angle = current_angle

hg = HistogramGrid(hg_dim[0], hg_dim[1])
hg.grid = from_map("map.txt")

ph = None

steps = []
index = 0

print "VARS INITIALIZED STARTING LOOP"
while index < MAX_STEPS:
    print "STEP", index
    #hg.print_hg(list(map(lambda s: s[1], steps)), start, end, current)
    if current == end: break

    ph = VFH.map_active_hg_to_ph(hg, PolarHistogram(nsectors), current, w_s, mhp_a, mhp_b, mhp_l)

    target_angle = wrap_angle(math.degrees(math.atan2(current[0] - end[0], current[1] - end[1])))
    best_angle = VFH.get_best_direction(ph.polar_histogram, target_angle, current_angle, previous_angle, gbd_t, gbd_smax, gbd_a, gbd_b, gbd_c)
    print "best_angle", best_angle
    steps.append((index, current, target_angle, wrap_angle(best_angle), ph.polar_histogram))
    
    # Compute next adjacent cell robot will be in 
    next_angle = math.floor(wrap_angle(best_angle + 22.5) / 45) * 45
    print "current", current
    print "next_angle", next_angle
    next_x = int((math.sqrt(2) if next_angle % 90 != 0 else 1) * math.cos(math.radians(next_angle + 90))) + current[0]
    next_y = int((math.sqrt(2) if next_angle % 90 != 0 else 1) * math.sin(math.radians(next_angle + 90))) * -1 + current[1]
    print "next x %d y %d" % (next_x, next_y)

    current = (next_x, next_y)
    previous_angle = current_angle
    current_angle = best_angle

    index += 1
    print "-" * 16

print "COMPLETE"
for s in steps:
    print "{0:2}. ({1:2}, {2:<2}) target_angle: {3:5.1f}   best_angle: {4:5.1f}".format(s[0], s[1][0], s[1][1],  s[2], s[3])

hg.print_hg(list(map(lambda s: s[1], steps)), start, end, current)







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
