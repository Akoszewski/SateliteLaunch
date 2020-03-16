import math
import random
import json
import os
from easygraphics import *
import matplotlib.pyplot as plt

maxTheta = 2*math.pi
planetRadius = 0.012
planetOmega = maxTheta

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
        if not os.path.isfile("planets.json"):
            r = 50
            for i in range(num):
                theta = round(random.uniform(0, maxTheta), 3)
                self.planets.append(Planet(r, theta))
                r = round(r * factor)
                self.dump()
        else:
            self.fromFile()

    def print(self):
        i = 1
        for planet in self.planets:
            print("Planet " + str(i) + ":")
            print("\tr = ", planet.r)
            print("\ttheta = ", planet.theta)
            print("\tperiod = ", planet.period)
            i = i + 1

    def dump(self):
        with open("planets.json", 'w') as f:
            serialized_planets = []
            for p in self.planets:
                serialized_planets.append({"r":p.r, "theta":p.theta, "period":p.period})
            f.write(str(json.dumps(serialized_planets)))

    def fromFile(self):
        with open("planets.json", 'r') as f:
            content = f.read()
            planets = json.loads(content)
            for p in planets:
                self.planets.append(Planet(p["r"], p["theta"]))

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

    def distanceToPixels(self, r):
        return r * (0.45 * self.ry / self.system.planets[-1].r)

    def __drawFrame(self, time):
        for p in self.system.planets:
            r = self.distanceToPixels(p.r)
            circle(self.rx/2, self.ry/2, r)
            theta = p.calculateTheta(time)
            [x, y] = convToCartesian(r, theta)
            draw_circle(self.rx/2 + x, self.ry/2 + y, 5)
        if self.satellite.vr != 0 and self.distanceToPixels(self.satellite.r) < self.rx: # optimization
            self.trace.append(self.satellite.calculatePosition(time))
        for pos in self.trace:
            [x, y] = pos
            x = self.distanceToPixels(x)
            y = self.distanceToPixels(y)
            draw_circle(self.rx/2 + x, self.ry/2 + y, 1)

class Satellite:
    def __init__(self, planet, speed, angle):
        self.r = planet.r
        self.theta = planet.theta
        self.vr = speed
        self.vth = 2*math.pi/planet.period

    def calculatePosition(self, time):
        [x, y] = convToCartesian(self.r, self.theta)
        vx = self.vr*math.cos(self.theta) - self.r*self.vth*math.sin(self.theta)
        vy = self.vr*math.sin(self.theta) + self.r*self.vth*math.cos(self.theta)
        vx += - planetRadius*planetOmega*math.sin(0)
        vy += planetRadius*planetOmega*math.cos(0)
        # print(math.sqrt(vx**2+vy**2))
        ax = 0
        ay = 0
        return [x + vx*time + ax*time**2/2, y + vy*time + ay*time**2/2]

def calcDistance(r1, theta1, r2, theta2):
    return math.sqrt(r1**2 + r2**2 - 2*r1*r2*math.cos(abs(theta2 - theta1)))

def convToCartesian(r, theta):
    return [r * math.cos(theta), r * math.sin(theta)]

system = System(5, 1.618)
satellite = Satellite(system.planets[2], speed = 1, angle = 0)
Animation = Animation(system)
Animation.run(satellite, time = 0)

# plt.plot(dists)
# plt.ylabel('time')
# plt.ylabel('distance')
# plt.show()