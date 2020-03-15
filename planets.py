import math
import random
from easygraphics import *
import matplotlib.pyplot as plt

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

class Animation:
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
                # dists.append(round(calcDistance(self.system.planets[1].r, self.system.planets[1].calculateTheta(time), self.system.planets[3].r, self.system.planets[3].calculateTheta(time)), 3))
                if has_kb_hit():
                    isAnimation = True
                clear_device()
                self.__drawFrame(time)
                if isAnimation:
                    time = time + 1
                else:
                    draw_text(30, 30, 'Press any key to animate')
        close_graph()

    def calcPxRadius(self, r):
        return r * (0.45 * self.ry / self.system.planets[-1].r)

    def __drawFrame(self, time):
        for p in self.system.planets:
            r = self.calcPxRadius(p.r)
            circle(self.rx/2, self.ry/2, r)
            theta = p.calculateTheta(time)
            draw_circle(self.rx/2 + r*math.cos(theta), self.ry/2 + r*math.sin(theta), 5)
        if self.satellite.vr != 0 and self.calcPxRadius(self.satellite.r) < self.rx: # optimization
            self.trace.append(self.satellite.calculatePosition(time))
        for pos in self.trace:
            [r, theta] = pos
            r = self.calcPxRadius(r)
            draw_circle(self.rx/2 + r*math.cos(theta), self.ry/2 + r*math.sin(theta), 1)

class Satellite:
    def __init__(self, planet, speed, angle):
        self.r = planet.r
        self.theta = planet.theta
        self.vr = speed
        self.vth = 2*math.pi/planet.period

    def calculatePosition(self, time):
        return [self.r + self.vr * time, self.theta + self.vth * time]

def calcDistance(r1, theta1, r2, theta2):
    return math.sqrt(r1**2 + r2**2 - 2*r1*r2*math.cos(abs(theta2 - theta1)))

system = System(5, 1.618)
satellite = Satellite(system.planets[2], speed = 1, angle = 0)
Animation = Animation(system)
Animation.run(satellite, time = 0)

# plt.plot(dists)
# plt.ylabel('time')
# plt.ylabel('distance')
# plt.show()