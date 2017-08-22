from __future__ import division
import math, time, random


class Math:

    @staticmethod
    def dist(p1, p2):
        return math.sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1])+(p1[2]-p2[2])*(p1[2]-p2[2]))


class Timer:

    def __init__(self):
        self.current_time = time.time()

    def elapsed(self):
        return time.time() - self.current_time

    def reset(self):
        self.current_time = time.time()
        

class RRTAgent:

    def __init__(self, start=(0, 0, 0), goal=(50, 5, 50),
                 xdims=(-50, 50), ydims=(-5, 5), zdims=(-50, 50),
                 alpha=0.1, beta=3.25, epsilon=7.0,
                 xrad=2.0, yrad=2.0, zrad=2.0,
                 move_time=0.5, path_time=0.5):
        if alpha < 0 or alpha >= 1: raise ValueError
        if beta <= 0: raise ValueError
        if epsilon <= 0: raise ValueError
        if xrad <= 0: raise ValueError
        if yrad <= 0: raise ValueError
        if zrad <= 0: raise ValueError
        self.position = start
        self.goal = goal
        self.dimensions = (xdims, ydims, zdims)
        self.alpha = alpha
        self.beta = beta
        self.epsilon = epsilon
        self.radii = xrad, yrad, zrad
        self.move_time = move_time
        self.path_time = path_time
        self.pathing_timer = Timer()
        self.move_timer = Timer()

    def get_xmin(self):
        return min(self.dimensions[0])

    def get_xmax(self):
        return max(self.dimensions[0])

    def get_ymin(self):
        return min(self.dimensions[1])

    def get_ymax(self):
        return max(self.dimensions[1])

    def get_zmin(self):
        return min(self.dimensions[2])

    def get_zmax(self):
        return max(self.dimensions[2])


    def in_bounds(self, p):
        x, y, z = p
        if x < self.get_xmin() or x > self.get_xmax():
            return False
        if y < self.get_ymin() or y > self.get_ymax():
            return False
        if z < self.get_zmin() or z > self.get_zmax():
            return False
        return True

    def ellipsoid(self):
        return (random.uniform(-self.radii[0], self.radii[0]),
                random.uniform(-self.radii[1], self.radii[1]),
                random.uniform(-self.radii[2], self.radii[2]))

    def line_to(self, p1, p2):
        if Math.dist(p1, p2) < self.epsilon:
            return p2
        else:
            theta = math.atan2(p2[1]-p1[1], p2[0]-p1[0])
            mx, my = math.cos(theta), math.sin(theta)
            return p1[0] + mx * self.epsilon, p1[1] + my * self.epsilon, p2[1] + (my/mx) * self.epsilon

    def sample(self):
        p = random.random()
        if p <= 1-self.alpha:
            return self.line_to()
        # TODO: This needs a los check added to it
        elif p <= (1-self.alpha)/self.beta:
            return self.uniform()
        else:
            return self.ellipsoid()

    def uniform(self):
        return (random.uniform(self.get_xmin(), self.get_xmax()),
                random.uniform(self.get_ymin(), self.get_ymax()),
                random.uniform(self.get_zmin(), self.get_zmax()))