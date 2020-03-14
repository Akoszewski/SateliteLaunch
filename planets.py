import math
import random
from easygraphics import *

maxTheta = 1

class Planet:
    def __init__(self, r, theta):
        self.r = r
        self.theta = theta
        c = 0.06
        self.period = round(math.sqrt(c * r**3)) # from Third Kepler's Law

    def calculateTheta(self, time):
        return (self.theta + time/self.period) % maxTheta

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
            
    def draw(self):
        rx = 800
        ry = 600
        r = 0.45 * ry
        # self.planets[-1].r
        # set_fill_color(Color.LIGHT_BLUE)
        init_graph(rx, ry)
        set_fill_color(Color.BLACK)
        for p in self.planets:
            r = p.r*(0.45*ry/self.planets[-1].r)
            circle(rx/2, ry/2, r)
            draw_circle(rx/2 + r*math.cos(p.theta*2*math.pi), ry/2 + r*math.sin(p.theta*2*math.pi), 5)
        while is_run():
            x = 1
        close_graph()

system = System(5, 1.618)
system.print()
system.draw()
# for t in range(80):s
for i in range(len(system.planets)):
    print(round(system.planets[i].calculateTheta(1556), 3))
