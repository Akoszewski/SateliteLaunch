import math
import random
import json
import os
from easygraphics import *
import matplotlib.pyplot as plt

maxTheta = 2*math.pi
planetRadius = 0.012 # comparable to Earth radius, satellite starts from here (not from the center)
planetOmega = maxTheta

GM = 0.00297 # gravity constant times mass
isStar = True

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
        if isStar == True:
            [px, py] = [0, 0]
            distance = calculateDistanceCart(self.satellite.x, self.satellite.y, px, py)
            cos = (self.satellite.x - px)/distance
            sin = (self.satellite.y - py)/distance
            a = 989/(distance**2)
            ax += a*cos
            ay += a*sin
        self.satellite.vx -= ax*(interval**2)/2
        self.satellite.vy -= ay*(interval**2)/2
        [self.satellite.x, self.satellite.y] = self.satellite.calculatePosition(interval) # get position next day

    def SimulateFlight(self, missionTime, target):
        bestDist = 999999
        for time in range(missionTime):
            if time >= self.satellite.t0:
                self.updateSatellite(time, 1)
            [tx, ty] = convToCartesian(target.r, target.calculateTheta(time))
            distance = calculateDistanceCart(self.satellite.x, self.satellite.y, tx, ty)
            if (bestDist > distance):
                bestDist = distance
        return bestDist
        

class Satellite:
    def __init__(self, planet, t0, speed, angle):
        self.r = planet.r + planetRadius
        self.theta = planet.theta
        self.vr = speed
        self.t0 = t0
        [self.x, self.y] = convToCartesian(self.r, self.theta)
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

def randomGaussian(mu, sigma): # randomizing with Box-Muller transform
    r1 = random.random()
    r2 = random.random()
    z1 = math.sqrt(-2 * math.log(r1)) * math.sin(2*math.pi*r2)
    z2 = math.sqrt(-2 * math.log(r1)) * math.cos(2*math.pi*r2)
    x1 = mu + z1 * sigma
    x2 = mu + z2 * sigma
    return x1

 #### THE ALGORITHM ####

def genAngleStep():
    return maxAngle/iterations * 20
def genSpeedStep():
    return maxSpeed/iterations * 20
def genDelayStep():
    return maxDelay/iterations * 20

def mutate(x, maxx, sigma):
    mutated = x + randomGaussian(0, sigma)
    if mutated > maxx:
        mutated = maxx
    if mutated < 0:
        mutated = 0
    return mutated
    
def cooling(temp, initTemp, iterations):
    t = temp - initTemp/iterations
    if temp % 10 == 0:
        print("Temp: " + str(t))
    return t

missionTime = 800
iterations = 500
maxAngle = 2*math.pi
maxSpeed = 1.5
maxDelay = missionTime

initialTemp = 5000

system = System(5, 1.618)
speed = maxSpeed/2
angle = maxAngle/2
t0 = 0
dists = []

angles = []
speeds = []
t0s = []

speedBest = speed
angleBest = angle
t0Best = t0

target = 1
startingPoint = 3

targetDist = 4 # minimal distance to fly near the target planet (stops the algorithm)

T = initialTemp
for iteration in range(iterations):
    if T == 0:
        break
    if len(dists) > 0:
        if dists[-1] <= targetDist:
            break

    speedMutated = mutate(speed, maxSpeed, genSpeedStep())
    angleMutated = mutate(angle, maxAngle, genAngleStep())
    t0Mutated = mutate(t0, maxDelay, genDelayStep())

    satellite = Satellite(system.planets[startingPoint], t0 = t0Mutated, speed = speedMutated, angle = angleMutated)
    animation = Animation(system, satellite)
    bestDistT = animation.SimulateFlight(missionTime, system.planets[target])

    satellite = Satellite(system.planets[startingPoint], t0 = t0, speed = speed, angle = angle)
    animation = Animation(system, satellite)
    bestDistX = animation.SimulateFlight(missionTime, system.planets[target])

    satellite = Satellite(system.planets[startingPoint], t0 = t0Best, speed = speedBest, angle = angleBest)
    animation = Animation(system, satellite)
    bestDistBestargs = animation.SimulateFlight(missionTime, system.planets[target])

    if bestDistT < bestDistX or randomGaussian(0, 1) < math.exp((bestDistX - bestDistT)/T):
        speed = speedMutated
        angle = angleMutated
        t0 = t0Mutated

    satellite = Satellite(system.planets[startingPoint], t0 = t0, speed = speed, angle = angle)
    animation = Animation(system, satellite)
    bestDistX = animation.SimulateFlight(missionTime, system.planets[target])

    if bestDistX < bestDistBestargs:
        speedBest = speed
        angleBest = angle
        t0Best = t0

    T = cooling(T, initialTemp, iterations)

    dists.append(bestDistBestargs)

    angles.append(angleBest)
    speeds.append(speedBest)
    t0s.append(t0Best)

    if (iteration % 10) == 0:
        print(iteration)

plt.plot(dists)
plt.xlabel('iteration')
plt.ylabel('best distance')
plt.show()

plt.plot(angles)
plt.xlabel('iteration')
plt.ylabel('best angle')
plt.show()

plt.plot(speeds)
plt.xlabel('iteration')
plt.ylabel('best speed')
plt.show()

plt.plot(t0s)
plt.xlabel('iteration')
plt.ylabel('best t0')
plt.show()

satellite = Satellite(system.planets[startingPoint], t0 = t0s[-1], speed = speeds[-1], angle = angles[-1])
print("Distance: " + str(dists[-1]) + " launch time: " + str(t0s[-1]) + " speed: " + str(speeds[-1]) + " angle: " + str(angles[-1]))
animation = Animation(system, satellite)
animation.run(0)

# system.print() # prints the planets
