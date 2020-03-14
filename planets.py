import math
import random

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


system = System(5, 1.618)
system.print()
# for t in range(80):
for i in range(len(system.planets)):
    print(round(system.planets[i].calculateTheta(1556), 3))
