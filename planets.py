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
        return (self.theta + time/self.period*maxTheta) % maxTheta

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
    def __init__(self, system):
        self.system = system

    def run(self, time, angle, speed, origin):
        rx = 800
        ry = 600
        r = 0.45 * ry
        isAnimation = True
        init_graph(rx, ry)
        set_render_mode(RenderMode.RENDER_MANUAL)
        set_fill_color(Color.BLACK)
        while is_run():
            if delay_jfps(40):
                clear_device()
                self.__drawFrame(rx, ry, time)
                if isAnimation:
                    time = time + 1
        close_graph()

    def __drawFrame(self, rx, ry, time):
        for p in self.system.planets:
            r = p.r*(0.45*ry/self.system.planets[-1].r)
            circle(rx/2, ry/2, r)
            theta = p.calculateTheta(time)
            draw_circle(rx/2 + r*math.cos(theta), ry/2 + r*math.sin(theta), 5)

system = System(5, 1.618)
simulation = Simulation(system)
simulation.run(time = 0, angle = 0, speed = 1, origin = 2)
#system.print()
# for t in range(80):
# for i in range(len(system.planets)):
    # print(round(system.planets[i].calculateTheta(1556), 3))
