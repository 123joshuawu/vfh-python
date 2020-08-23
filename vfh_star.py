import math
import heapq
import numpy as np
from itertools import groupby, count
import operator
from decimal import Decimal, ROUND_UP

import utm
from itertools import starmap, chain
from shapely.geometry.polygon import Polygon
from shapely.geometry import Point

import matplotlib
matplotlib.use('TkAgg')

def get_line(x1, y1, x2, y2):
    points = []
    issteep = abs(y2-y1) > abs(x2-x1)
    if issteep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    rev = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        rev = True
    deltax = x2 - x1
    deltay = abs(y2-y1)
    error = int(deltax / 2)
    y = y1
    ystep = None
    if y1 < y2:
        ystep = 1
    else:
        ystep = -1
    for x in range(x1, x2 + 1):
        if issteep:
            points.append((y, x))
        else:
            points.append((x, y))
        error -= deltay
        if error < 0:
            y += ystep
            error += deltax
    # Reverse the list if the coordinates were reversed
    if rev:
        points.reverse()
    return points

def trace_bounds(hg, bounds):

    for i in range(len(bounds) - 1):
        # print("BOUNDS", bounds[i], bounds[i + 1])
        for p in get_line(bounds[i][0], bounds[i][1], bounds[i + 1][0], bounds[i + 1][1]):
            hg.set_certainty(p[0], p[1], 15)


def bound(index, minimum, maximum):
    if index < minimum:
        return 0
    elif index >= maximum:
        return maximum
    else:
        return index


class HistogramGrid:
    """ Class HistogramGrid defines a nested array ("grid") of certainty values
        Coordinate points start from 0
    """

    def __init__(self, ncols, nrows, bounds):
        self.histogram_grid = [[0 for c in range(ncols)] for r in range(nrows)]
        self.poly = Polygon(bounds)
        self.bounds = self.poly.bounds
        self.dimensions = (nrows, ncols)
        trace_bounds(self, bounds)

    def out_of_bounds(self, x, y):
        """ Returns whether the cell is out of the grid. Used for edge conditions """
        return not self.poly.contains(Point(x, y))
        # return 0 > y or y >= len(self.histogram_grid) or 0 > x or x >= len(self.histogram_grid[0])

    def bound_x(self, x):
        return int(bound(x, self.bounds[0], self.bounds[2]))

    def bound_y(self, y):
        return int(bound(y, self.bounds[1], self.bounds[3]))

    def get_certainty(self, x, y):
        return self.histogram_grid[len(self.histogram_grid) - y - 1][x]

    def set_certainty(self, x, y, certainty):
        self.histogram_grid[len(self.histogram_grid) - y - 1][x] = certainty

    def get_obstacles(self):
        row = len(self.histogram_grid[0])
        return list(map(lambda t: ((t[0] // row, t[0] % row), t[1]), filter(lambda c: c[1] > 0, enumerate(chain(*self.histogram_grid)))))
#        obs = []
#        for y in range(int(self.bounds[3])+1):
#            for x in range(int(self.bounds[2])+1):
#                c = self.get_certainty(x, y)
#                if c > 0:
#                    obs.append(((x, y), c))
#        return obs

    def print_hg(self, robot_locations, start, end, current, o_nodes=[], c_nodes=[], e_nodes=[], b_nodes=[], window_radius=None):
        """ For testing purposes """
        string = ""

        if window_radius is None:
            window_radius = len(self.histogram_grid)
        if current is None: 
            current = start
        for y in reversed(range(self.bound_y(current[1] - window_radius), self.bound_y(current[1] + window_radius) + 1)):
            for x in range(self.bound_x(current[0] - window_radius), self.bound_x(current[0] + window_radius) + 1):

                if self.get_certainty(x, y) == 1:
                    string += "1 "
                elif (x, y) == current:
                    string += "C "
                elif (x, y) == start:
                    string += "S "
                elif (x, y) == end:
                    string += "E "    
                elif (x, y) in b_nodes:
                    string += "B "
                elif (x, y) in e_nodes:
                    string += ". "
                elif (x, y) in c_nodes:
                    string += "# "  
                elif (x, y) in o_nodes:
                    string += "N " 
                elif (x, y) in robot_locations:
                    string += "X "
                else:
                    string += "0 "
            string += "\n"
        #string += "0/1 - Free/Occupied (Certainty values)\nX - Robot locations\nS - Start Position (%d, %d)\nE - End Target (%d, %d)\nC - Current" % (start[0], start[1], end[0], end[1])
        return string[:-1]

    def print_active_region(self, min_ax, max_ax, min_ay, max_ay, vcp):
        ar_string = ""

        for y in reversed(range(min_ay, max_ay + 1)):
            for x in range(min_ax, max_ax + 1):
                ar_string += "{} ".format(self.get_certainty(x, y)) if (x, y) != vcp else "X "
            ar_string += "\n"
        return ar_string[:-1]


class PolarHistogram:

    def __init__(self, nsectors):
        self.polar_histogram = [0 for s in range(nsectors)]
        self.nsectors = nsectors
        self.sector_angle = 360 / nsectors

    def add_certainty(self, sector, certainty):
        while sector >= len(self.polar_histogram):
            sector -= len(self.polar_histogram)

        self.polar_histogram[sector] += certainty

    def get_sector_certainty(self, sector):
        return self.polar_histogram[sector]

    def __str__(self):
        """ Testing purposes """
        string = ""
        for tup in enumerate(self.polar_histogram):
            if tup[1] != 0:
                string += "{:<3} {}\n".format(tup[0] * self.sector_angle, tup[1])
        return string


def wrap_angle(angle):
    while angle > 360:
        angle -= 360

    while angle < 0:
        angle += 360

    return angle


def angle_between(n, a, b):
    n = wrap_angle(n)
    a = wrap_angle(a)
    b = wrap_angle(b)

    if (a < b): 
        return a <= n <= b
    else:
        return a <= n or n <= b


def small_angle_diff(a1, a2):
    """ Helper function for getting smallest angle difference between two angles """
    return abs((a1 - a2 + 180) % 360 - 180)


#------------------ VFH FUNCTIONS --------------------#

def get_polar_histogram(hg, vcp, w_s, n, rr, ds, t_low, t_high, a, b):

    ph = PolarHistogram(n)

    robot_x, robot_y = vcp
    window_radius = (w_s - 1) // 2

    min_active_x = hg.bound_x(robot_x - window_radius)
    max_active_x = hg.bound_x(robot_x + window_radius)
    min_active_y = hg.bound_y(robot_y - window_radius)
    max_active_y = hg.bound_y(robot_y + window_radius)

    print("Active Region -- X marks the robot")
    print(hg.print_active_region(min_active_x, max_active_x, min_active_y, max_active_y, vcp))

    for x in range(min_active_x, max_active_x + 1):
        for y in range(min_active_y, max_active_y + 1):

            dy = y - robot_y
            dx = x - robot_x
            cell_certainty = hg.get_certainty(x, y)

            if cell_certainty == 0 or (x, y) == vcp: continue

            cell_angle = wrap_angle(math.degrees(math.atan2(dy, dx)))
            cell_distance = math.hypot(dx, dy)

            cell_magnitude = (cell_certainty ** 2) * (a - b * cell_distance)
            # cell_magnitude = (cell_certainty ** 2) * (a - b * (cell_distance ** 2))

            if cell_distance < rr + ds: 
                raise Exception("Robot is too close to obstacle.")

            obstacle_enlargement_angle = math.degrees(math.asin((rr + ds) / cell_distance))
            # print("enlargement angle", obstacle_enlargement_angle)

            min_sector = int(math.floor((cell_angle - obstacle_enlargement_angle) / ph.sector_angle))
            max_sector = int((cell_angle + obstacle_enlargement_angle) / ph.sector_angle) + 1

            print("({0:<2}, {1:<2}) {5:>3}/{6:<3} = {2:6.1f} deg -- Distance: {3:5.1f} Certainty: {7} Magnitude: {4:.1f}".format(x, y, cell_angle, cell_distance, cell_magnitude, dy, dx, cell_certainty))

            for sector in range(min_sector, max_sector):
                # print("sector", sector * ph.sector_angle)

                ph.add_certainty(sector, cell_magnitude)

    print("\nPolarHistogram:\n" + str(ph))

    binary_ph = [0] * n
    offset = n - ph.polar_histogram.index(max(ph.polar_histogram)) - 1
    i = 0

    for c in np.roll(ph.polar_histogram, offset):
        if c > t_high:
            binary_ph[i] = 1
        elif c < t_low:
            binary_ph[i] = 0
        else:
            binary_ph[i] = binary_ph[i - 1] 
        i += 1

    ph.polar_histogram = np.roll(binary_ph, -1 * offset)

    print("Binary PH\n" + str(ph))

    return ph



def get_candidate_angles(ph, smax, tgt_angle):

    polar_histogram = ph.polar_histogram

    if not 0 in polar_histogram: 
        raise Exception("All sectors occupied, no possible directions")

    if max(polar_histogram) == 0:
        return [tgt_angle]

    valleys = [list(g) for k, g in groupby(list(enumerate(polar_histogram)), operator.itemgetter(1))]
    print("Valleys", valleys)

    if valleys[0][0][1] == valleys[-1][0][1]:
        valleys[-1].extend(map(lambda tup: (tup[0] + len(polar_histogram), tup[1]), valleys.pop(0)))
        print("Wrapped last valley")

    free_valleys = filter(lambda v: v[0][1] == 0, valleys)
    print("Free valleys", free_valleys)


    print("Generating candidate angles")
    print("Target Angle", tgt_angle)
    candidate_angles = []

    for v in free_valleys:
        if v[-1][0] - v[0][0] > smax:
            left_candidate = wrap_angle((v[0][0] + smax / 2) * ph.sector_angle)
            right_candidate = wrap_angle((v[-1][0] - smax / 2) * ph.sector_angle)
            candidate_angles.append(left_candidate)
            candidate_angles.append(right_candidate)
            print("Wide Valley (%d, %d)... Adding %d and %d" % (v[0][0] * ph.sector_angle, v[-1][0] * ph.sector_angle, candidate_angles[-2], candidate_angles[-1]))

            if angle_between(tgt_angle, left_candidate, right_candidate):
                print("Target in btwn (%d, %d)... Adding %d" % (left_candidate, right_candidate, tgt_angle))
                candidate_angles.append(tgt_angle)
        else:
            candidate_angles.append(wrap_angle(((v[0][0] + v[-1][0]) / 2) * ph.sector_angle)) # Add middle of valley to candidat angles
            print("Narrow Valley (%d, %d)... Adding %d" % (v[0][0] * ph.sector_angle, v[-1][0] * ph.sector_angle, candidate_angles[-1]))

    return candidate_angles



def primary_cost(candidate_angle, cur_angle, prev_angle, tgt_angle, a, b, c):
    return a * small_angle_diff(candidate_angle, tgt_angle) \
            + b * small_angle_diff(candidate_angle, cur_angle) \
            + c * small_angle_diff(candidate_angle, prev_angle)



def projected_cost(candidate_angle, effective_direction, cur_angle, prev_angle, tgt_angle, discount_factor, a, b, c):
    return discount_factor * (a * max(small_angle_diff(candidate_angle, tgt_angle), \
                                        small_angle_diff(effective_direction, tgt_angle)) \
                                + b * small_angle_diff(candidate_angle, cur_angle) \
                                + c * small_angle_diff(candidate_angle, prev_angle))



def heuristic(cur_angle, prev_angle, tgt_angle, discount_factor, b, c):
    return discount_factor * (b * small_angle_diff(tgt_angle, cur_angle) \
                                + c * small_angle_diff(tgt_angle, prev_angle))


# Computationally more expensive but more accurate
# def heuristic(effective_direction, cur_angle, prev_angle, discount_factor, a, b, c):
#     return discount_factor * (a * small_angle_diff(effective_direction, tgt_angle) \
#                                 + b * small_angle_diff(tgt_angle, cur_angle) \
#                                 + c * small_angle_diff(tgt_angle, prev_angle))



def get_projected_location(x, y, angle, d_s):
    # Disregarding dynamics of robot
    if angle % 90 == 0:
        projected_x = x + int(round(math.cos(math.radians(angle)) * d_s))
        projected_y = y + int(round(math.sin(math.radians(angle)) * d_s))
    else:
        projected_x = x + round_away_zero(math.cos(math.radians(angle)) * d_s)
        projected_y = y + round_away_zero(math.sin(math.radians(angle)) * d_s)

    return (projected_x, projected_y)



def get_projected_angle(cur_angle, tgt_angle):
    # Disregarding dynamics of robot
    return tgt_angle


#--------------------- VFH STAR FUNCTIONS ---------------- #

def a_star(hg, start, start_angle, end, d_t, d_s, w_s, n, rr, ds, t_low, t_high, smax, cm_a, cm_b, a, b, c, mu1, mu2, mu3, discount_factor, steps):
    n_g = d_t // d_s

    open_nodes = PriorityQueue()
    closed_nodes = []
    
    end_nodes = []
    previous_locations = set([])
    total_costs = []

    open_nodes.put(PrimaryNode(start[0], start[1], start_angle, hg, n_g, end, a, b, c), 0)
    previous_locations.add(start)

    while not open_nodes.empty():
        costs = []
        current = open_nodes.get()
        closed_nodes.append(current)

        previous_locations.add(current.location)

        if current.location == end:
            end_nodes.append(current) 
            break

        if hg.out_of_bounds(current.x, current.y) or small_angle_diff(current.cur_direction, current.get_prev_angle()) > 120:
            if current in end_nodes: del end_nodes[-1]
            continue

        if current.depth == 0:
            end_nodes.append(current)
            continue

        # try:
        pph = current.projected_polar_histogram(w_s, n, rr, ds, t_low, t_high, cm_a, cm_b)
        candidate_angles = current.projected_candidate_directions(smax)
        print("candidate angles", candidate_angles)
        # except Exception as e:
        #     print("PROBALBY COLLISION " + repr(e))
        #     if current in end_nodes: 
        #         del end_nodes[-1]
        #     continue

        children = [None for i in range(len(candidate_angles))]
        i = 0

        for ca in candidate_angles:
            child_orientation = current.get_projected_angle(ca)
            child_x, child_y = current.get_projected_location(ca, d_s, end)
            # print("CA %d deg -- Child: (%d, %d)" % (ca, child_x, child_y) 

            cost = current.projected_cost(ca, discount_factor, mu1, mu2, mu3)
            costs.append(cost)

            if (child_x, child_y) not in previous_locations or cost < current.cost:
                # print("NEW CHILD " + str((child_x, child_y))
                child = ProjectedNode(ca, child_x, child_y, child_orientation, current, hg, current.depth - 1, end, cost, cost + current.total_cost)
                priority = cost + child.heuristic(discount_factor, mu2, mu3)
                open_nodes.put(child, priority)
                children[i] = child.location
            i += 1


        # print(hg.print_hg(list(map(lambda s: s[1], steps)), start, end, current.location, open_nodes.get_locs(), previous_locations, list(map(lambda e: e.location, end_nodes)), [], (w_s - 1) / 2))
        # print("Current " + str(current.location))
        # print("Candidate angles " + str(candidate_angles))
        # print("Costs " + str(costs)) 
        # print("CUR COST: " + str(current.cost))
        # print("Children " + str(children))
        # print("Prev locs " + str(previous_locations))
        # raw_input()

    print(closed_nodes)
    print(end_nodes)
    locs = []
    if not end_nodes:
        primary_candidate_angle, locs = closed_nodes[0].get_primary_candidate_angle()
    else:
        primary_candidate_angle, locs = min(end_nodes, key=operator.attrgetter('total_cost')).get_primary_candidate_angle()
    
    # print(hg.print_hg(list(map(lambda s: s[1], steps)), start, end, None, open_nodes.get_locs(), previous_locations, list(map(lambda e: e.location, end_nodes)), locs, (w_s - 1) / 2))
    # print("End nodes " + str(list(map(lambda e: e.location, end_nodes))))
    # print("Total costs " + str(list(map(lambda e: int(e.total_cost), end_nodes))))
    # print("Best nodes " + str(locs))
    # print("Best angle " + str(primary_candidate_angle))
    # print("Start " + str(start))
    # raw_input()

    return primary_candidate_angle


class PriorityQueue:

    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

    def get_locs(self):
        return list(map(lambda n: n[1].location, self.elements))


def round_away_zero(num):
    """ Specially used for projected location, to be removed once robot dynamics added """
    return int(Decimal(num).quantize(Decimal('1.'), ROUND_UP))


class Node(object):

    def __init__(self, x, y, hg, cur_direction, depth, cost, total_cost):
        self.x = x
        self.y = y
        self.location = (x, y)
        self.hg = hg
        self.cur_direction = cur_direction
        self.depth = depth
        self.cost = cost
        self.total_cost = total_cost

    def get_projected_location(self, angle, d_s, end):
        if d_s > math.sqrt(2) and math.hypot(end[1] - self.y, end[0] - self.x) < d_s:
            return end

        return get_projected_location(self.x, self.y, angle, d_s)

    def get_projected_angle(self, tgt_angle):
        return get_projected_angle(self.cur_direction, tgt_angle)

    def projected_polar_histogram(self, w_s, n, rr, ds, t_low, t_high, a, b):
        self.pph = get_polar_histogram(self.hg, self.location, w_s, n, rr, ds, t_low, t_high, a, b)

        return self.pph

    def projected_candidate_directions(self, smax):
        self.pcd = get_candidate_angles(self.pph, smax, self.tgt_angle)

        return self.pcd

    def get_primary_candidate_angle(self, ca=None, prev_locs=None):
        pass


class ProjectedNode(Node):

    def __init__(self, candidate_angle, x, y, cur_direction, parent, hg, depth, end, cost, total_cost):
        super().__init__(x, y, hg, cur_direction, depth, cost, total_cost)

        self.candidate_angle = candidate_angle
        self.parent = parent
        self.effective_direction = wrap_angle(math.degrees(math.atan2(y - parent.y, x - parent.x)))
        self.tgt_angle = wrap_angle(math.degrees(math.atan2(end[1] - y, end[0] - x)))
        
    def projected_cost(self, candidate_angle, discount_factor, mu1, mu2, mu3):
        return discount_factor * (mu1 * max(small_angle_diff(candidate_angle, self.tgt_angle), \
                                            small_angle_diff(self.effective_direction, self.tgt_angle)) \
                                    + mu2 * small_angle_diff(candidate_angle, self.cur_direction) \
                                    + mu3 * small_angle_diff(candidate_angle, self.get_prev_angle()))


    def heuristic(self, discount_factor, mu2, mu3):
        return discount_factor * (mu2 * small_angle_diff(self.tgt_angle, self.cur_direction) \
                                    + mu3 * small_angle_diff(self.tgt_angle, self.get_prev_angle()))

    def get_prev_angle(self):
        return self.parent.cur_direction

    def get_primary_candidate_angle(self, ca=None, prev_locs=None):
        if ca is None: ca = -1
        if prev_locs is None: prev_locs = list()
        prev_locs.append(self.location)
        print("PRI_CAND (%d, %d) -- %d deg" % (self.x, self.y, ca))

        return self.parent.get_primary_candidate_angle(self.candidate_angle, prev_locs)


class PrimaryNode(Node):

    def __init__(self, x, y, cur_direction, hg, depth, end, a, b, c):
        super().__init__(x, y, hg, cur_direction, depth, 0, 0)

        self.tgt_angle = wrap_angle(math.degrees(math.atan2(end[1] - y, end[0] - x)))
        self.best_angle = None
        self.a = a
        self.b = b
        self.c = c

    def projected_candidate_directions(self, smax):
        cds = get_candidate_angles(self.pph, smax, self.tgt_angle)
        
        if len(cds) == 1:
            self.best_angle = cds[0]
            return []
        else:
            return cds

    def projected_cost(self, candidate_angle, discount_factor, mu1, mu2, mu3):
        return self.a * small_angle_diff(candidate_angle, self.tgt_angle) \
                + self.b * small_angle_diff(candidate_angle, self.cur_direction) \
                + self.c * small_angle_diff(candidate_angle, self.get_prev_angle())

    def get_prev_angle(self):
        return self.cur_direction

    def get_primary_candidate_angle(self, ca=None, prev_locs=None):
        print("GOT THE ANGLE ITS " + str(ca))
        if prev_locs is None: prev_locs = []

        return (ca, prev_locs) if ca else (self.best_angle, prev_locs)



# from mapping import *
# from pyproj import Proj, transform
# import utm
# from itertools import starmap
# from shapely.geometry import Polygon

# p_l = Proj(proj='latlong', datum='WGS84')
# p_m = Proj(proj='utm', zone=10, datum='NAD27')


def get_meter_poly(l_coords):
    m_coords = list(starmap(latlon_convert_to_meter, l_coords))
    zone = Polygon(m_coords)
    return zone


def latlon_convert_to_meter(lat, lng):
    return utm.from_latlon(lat, lng)[:2]
    # return transform(p_l, p_m, lat, lng)

def add_polar(v1, v2):
    r_x = math.cos(math.radians(v1[0])) * v1[1] + math.cos(math.radians(v2[0])) * v2[1]
    r_y = math.sin(math.radians(v1[0])) * v1[1] + math.sin(math.radians(v2[0])) * v2[1]
    r = math.hypot(r_x, r_y)
    a = wrap_angle(math.atan2(r_y, r_x))
    return (a, r)

class Zone:

    def __init__(self, name, coords, resolution):
        self.name = name
        self.poly = get_meter_poly(coords)
        self.resolution = resolution
        bounds = self.poly.bounds
        self.origin = (bounds[0], bounds[1])
        hg_bounds = list(starmap(self.get_cell_m, list(self.poly.exterior.coords)))

        self.hg = HistogramGrid(int((bounds[2] - bounds[0]) / resolution), int((bounds[3] - bounds[1]) / resolution), hg_bounds)

    def get_cell_latlng(self, lat, lng):
        return self.get_cell_m(*latlon_convert_to_meter(lat, lng))

    def get_cell_m(self, m_e, m_n):
        return (int((m_e - self.origin[0]) / self.resolution), int((m_n - self.origin[1]) / self.resolution))

    def get_points(self):
        return list(self.poly.exterior.coords)


class Sensor:

    def __init__(self, name, angle, radius, callback):
        self.name = name
        self.angle = angle
        self.radius = radius
        self.callback = callback

    def get_readings(self, robot_angle):
        t_readings = []
        for v_i in self.callback():
            v_f = add_polar(v_i, (self.angle - robot_angle, self.radius))
            x_g = int(math.floor(v_f[1] * math.cos(math.radians(v_f[0]))))
            y_g = int(math.floor(v_f[1] * math.sin(math.radians(v_f[0]))))
            t_readings.append((x_g, y_g))
        return t_readings

class Bot:

    def __init__(self, zone):
        self.sensors = []
        self.zone = zone

    def update_location(self, lat, lng, angle):
        robot_x, robot_y = self.zone.get_cell_latlng(lat, lng)

        self.aggregate_readings(robot_x, robot_y, angle)

    def aggregate_readings(self, x, y, angle):
        for r in zip(map(lambda s: s.get_readings(angle), self.sensors)):
            self.zone.hg.add_certainty(r[0], r[1])

        return True

    def add_sensor(self, name, angle, radius, callback):
        self.sensors.append(name, angle, radius, callback)

        return True












