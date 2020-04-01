import math
import random
import json
import os
import numpy as np
from easygraphics import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

maxTheta = 2*math.pi
planetRadius = 0.012
planetOmega = maxTheta

# GM = 0.00297 * 1000000
GM = 0.00297 # gravity constant times mass

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
    def __init__(self, system, satellite):
        self.system = system
        self.satellite = satellite

    def run(self, time):
        self.prevTime = time
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
                    self.prevTime = time
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
        if self.satellite.vr != 0 and self.distanceToPixels(math.sqrt(self.satellite.x**2 + self.satellite.y**2)) < self.rx: # optimization
            if time >= self.satellite.t0:
                self.updateSatellite(time, time - self.prevTime)
                self.trace.append([self.satellite.x, self.satellite.y])
        for pos in self.trace:
            [x, y] = pos
            x = self.distanceToPixels(x)
            y = self.distanceToPixels(y)
            draw_circle(self.rx/2 + x, self.ry/2 + y, 1)

    def updateSatellite(self, time, interval):
        [ax, ay] = [0, 0]
        for planet in self.system.planets:
            [px, py] = convToCartesian(planet.r, planet.calculateTheta(time))
            distance = calculateDistanceCart(self.satellite.x, self.satellite.y, px, py)
            cos = (self.satellite.x - px)/distance
            sin = (self.satellite.y - py)/distance
            a = GM/(distance**2)
            ax += a*cos
            ay += a*sin
        # [px, py] = [0, 0]
        # distance = calculateDistanceCart(self.satellite.x, self.satellite.y, px, py)
        # cos = (self.satellite.x - px)/distance
        # sin = (self.satellite.y - py)/distance
        # a = 989/(distance**2)
        # ax += a*cos
        # ay += a*sin
        self.satellite.vx -= ax*(interval**2)/2
        self.satellite.vy -= ay*(interval**2)/2
        [self.satellite.x, self.satellite.y] = self.satellite.calculatePosition(interval) # get position next day
        # print("position: " + str([self.satellite.x, self.satellite.y]))
        # print("velocity: " + str([self.satellite.vx, self.satellite.vy]))
        # print("acceleration: " + str([ax, ay]))
        

class Satellite:
    def __init__(self, planet, t0, speed, angle):
        self.r = planet.r + planetRadius
        self.theta = planet.theta
        self.vr = speed
        self.t0 = t0
        [self.x, self.y] = convToCartesian(self.r, self.theta)
        print("speed: " + str(speed))
        print("angle: " + str(angle))
        [self.vx, self.vy] = convToCartesian(self.vr, angle)

    def calculatePosition(self, time):
        return [self.x + self.vx * time, self.y + self.vy * time]

def convToCartesian(r, theta):
    return [r * math.cos(theta), r * math.sin(theta)]

def calculateDistance(satellite, planet, time):
    planet_angle = planet.theta + 2*math.pi/planet.period * time
    return math.sqrt((satellite.r*math.cos(satellite.theta) + satellite.vr*math.cos(satellite.theta)*time
        - planet.r*math.cos(planet_angle))**2 + (satellite.r*math.sin(satellite.theta) + 
        satellite.vr*math.sin(satellite.theta) - planet.r*math.sin(planet_angle))**2)

def calculateDistancePolar(r1, theta1, r2, theta2):
    return math.sqrt(r1**2 + r2**2 - 2*r1*r2*math.cos(theta2 - theta1))

def calculateDistanceCart(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

#3.6387 # 3 predkosc kosmiczna

missionTime = 500
iterations = 50
maxAngle = 2*math.pi
maxSpeed = 1.5
maxDelay = missionTime

# def genAngle(iteration):
#     return maxAngle/iterations * iteration
# def genSpeed(iteration):
#     return maxSpeed/iterations * iteration
#     # return 2
# def genDelay(iteration):
#     # return maxDelay/iterations * iteration
#     return 0

system = System(5, 1.618)
bestDists2D = []
speeds2D = []
angles2D = []
iterationT0 = 0 # will be used better later
# for iterationAngle in range(iterations):
#     bestDists = []
#     speeds = []
#     angles = []
#     for iterationSpeed in range(iterations):
#         dists = []
#         bestDist = 999999
#         target = system.planets[2]
#         satellite = Satellite(system.planets[4], t0 = genDelay(iterationT0), speed = genSpeed(iterationSpeed), angle = genAngle(iterationAngle))
#         animation = Animation(system, satellite)
#         for time in range(missionTime):
#             if time >= animation.satellite.t0:
#                 animation.updateSatellite(time, 1)
#             [tx, ty] = convToCartesian(target.r, target.calculateTheta(time))
#             distance = calculateDistanceCart(animation.satellite.x, animation.satellite.y, tx, ty)
#             if (bestDist > distance):
#                 bestDist = distance
#             dists.append(distance)
#         bestDists.append(bestDist)
#         speeds.append(genSpeed(iterationSpeed))
#         angles.append(genAngle(iterationAngle))
#     bestDists2D.append(bestDists)
#     speeds2D.append(speeds)
#     angles2D.append(angles)

# if iteration % 10 == 0:
#     print(str(iterationAngle + 1))
# fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.plot_surface(np.array(angles2D), np.array(speeds2D), np.array(bestDists2D))
# # plt.xlabel('angle')
# # plt.zlabel('distance')
# plt.show()

# satellite = Satellite(system.planets[4], t0 = 0, speed = 0.2, angle = 0)
# Animation = Animation(system, satellite)
# Animation.run(time = 0)

def randomGaussian(mu, sigma): # randomizing with Box-Muller transform
    r1 = random.random()
    r2 = random.random()
    z1 = math.sqrt(-2 * math.log(r1)) * math.sin(2*math.pi*r2)
    z2 = math.sqrt(-2 * math.log(r1)) * math.cos(2*math.pi*r2)
    x1 = mu + z1 * sigma
    x2 = mu + z2 * sigma
    return x1

def genAngleStep():
    return maxAngle/iterations
def genSpeedStep():
    return maxSpeed/iterations
def genDelayStep():
    return maxDelay/iterations

# gaussians = []
# for i in range(100):
#     gaussians.append(randomGaussian(0, 4))
#     # gaussians.append(random.random())
# plt.plot(gaussians)
# plt.xlabel('iteration')
# plt.ylabel('gaussian')
# plt.show()

def mutate(x, sigma):
    return x + randomGaussian(0, sigma)

speed = maxSpeed/2
angle = maxAngle/2
prevSpeed = speed
prevAngle = angle
t0 = 0
dists = []
for iteration in range(iterations):
    t0_array = []
    angles = []
    speeds = []
    # t0_array.append(missionTime/2)
    # speeds.append(maxSpeed/2)
    # angles.append(maxAngle/2)
    bestDist = 999999
    target = system.planets[2]
    satellite = Satellite(system.planets[4], t0 = 0, speed = speed, angle = angle)
    animation = Animation(system, satellite)
    for time in range(missionTime):
        if time >= animation.satellite.t0:
            animation.updateSatellite(time, 1)
        [tx, ty] = convToCartesian(target.r, target.calculateTheta(time))
        distance = calculateDistanceCart(animation.satellite.x, animation.satellite.y, tx, ty)
        if (bestDist > distance):
            bestDist = distance
    dists.append(bestDist)
    # t0_array.append(genDelay(dists, iteration, t0_array))

    if iteration > 0:
        if dists[iteration - 1] <= dists[iteration]:
            speed = mutate(speed, genSpeedStep())
            angle = mutate(speed, genAngleStep())
    else:
        speed = mutate(speed, genSpeedStep())
        angle = mutate(speed, genAngleStep())

    prevAngle = angle
    prevSpeed = speed
    # angles.append(genAngle(dists, iteration, angles))
    # speeds.append(genSpeed(dists, iteration, speeds))

plt.plot(dists)
plt.xlabel('iteration')
plt.ylabel('best distance')
plt.show()