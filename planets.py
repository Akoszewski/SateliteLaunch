import math
import random

class Planet:
    def __init__(self, r, theta):
        self.r = r
        self.theta = theta
        c = 0.06
        self.period = round(math.sqrt(c * r**3)) # from Third Kepler's Law
    
def generatePlanets(num, factor):
    planets = []
    r = 50
    for i in range(num):
        theta = round(random.uniform(0, 2 * math.pi), 3)
        planets.append(Planet(r, theta))
        r = round(r * factor)
    return planets

def printPlanets(planets):
    i = 1
    for planet in planets:
        print("Planet " + str(i) + ":")
        print("\tr = ", planet.r)
        print("\ttheta = ", planet.theta)
        print("\tperiod = ", planet.period)
        i = i + 1

t = 0
planets = generatePlanets(9, 1.618)
printPlanets(planets)