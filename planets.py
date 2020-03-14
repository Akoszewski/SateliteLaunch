import math
import random
from easygraphics import *

maxTheta = 2*math.pi

class Planet:
    def __init__(self, r, theta):
        self.r = r
        self.theta = theta
        c = 0.06
        self.period = round(math.sqrt(c * r**3)) # from Third Kepler's Law

    def calculateTheta(self, time):
        return (self.theta + time/self.period * maxTheta) % maxTheta

class System:
    planets = []
    def __init__(self, num, factor):
        r = 50
        for i in range(num):
            theta = round(random.uniform(0, maxTheta), 3)
            self.planets.append(Planet(r, theta))
            r = round(r * factor)

    def print(self):
        i = 1
        for planet in self.planets:
            print("Planet " + str(i) + ":")
            print("\tr = ", planet.r)
            print("\ttheta = ", planet.theta)
            print("\tperiod = ", planet.period)
            i = i + 1

class Simulation:
    rx = 800
    ry = 600
    trace = [] # satellite trace
    def __init__(self, system):
        self.system = system

    def run(self, satellite, time):
        self.satellite = satellite
        r = 0.45 * self.ry
        isAnimation = False
        init_graph(self.rx, self.ry)
        set_render_mode(RenderMode.RENDER_MANUAL)
        set_fill_color(Color.BLACK)
        while is_run():
            if delay_jfps(40):
                if has_kb_hit():
                    isAnimation = True
                clear_device()
                self.__drawFrame(time)
                if isAnimation:
                    time = time + 1
        close_graph()

    def calcPxRadius(self, r):
        return r * (0.45 * self.ry / self.system.planets[-1].r)

    def __drawFrame(self, time):
        for p in self.system.planets:
            r = self.calcPxRadius(p.r)
            circle(self.rx/2, self.ry/2, r)
            theta = p.calculateTheta(time)
            draw_circle(self.rx/2 + r*math.cos(theta), self.ry/2 + r*math.sin(theta), 5)
        if self.satellite.speed != 0:
            self.trace.append(self.satellite.calculatePosition(time))
        for pos in self.trace:
            [r, theta] = pos
            r = self.calcPxRadius(r)
            draw_circle(self.rx/2 + r*math.cos(theta), self.ry/2 + r*math.sin(theta), 1)

class Satellite:
    def __init__(self, r, theta, speed, angle):
        self.r = r
        self.theta = theta
        self.speed = speed

    def calculatePosition(self, time):
        return [self.r + self.speed * time, self.theta]

system = System(5, 1.618)
simulation = Simulation(system)
satellite = Satellite(system.planets[0].r, system.planets[0].theta, speed = 1, angle = 0)
simulation.run(satellite, time = 0)
#system.print()
# for t in range(80):
# for i in range(len(system.planets)):
    # print(round(system.planets[i].calculateTheta(1556), 3))
