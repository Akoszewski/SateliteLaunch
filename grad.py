import matplotlib.pyplot as plt

def f(x,y):
    return x**2 + x*y + y**2 - 3*x
def fprimx(x,y):
    return 2*x + y - 3
def fprimy(x,y):
    return x + 2*y

x = []
y = []
x.append(0)
y.append(0)
b = 0.1
for t in range(40):
    x.append(x[t] - b*fprimx(x[t], y[t]))
    y.append(y[t] - b*fprimy(x[t], y[t]))
    print("x[t] = " + str(x[t]) + " fx = " + str(fprimx(x[t], y[t])))
    print("y[t] = " + str(y[t]) + " fy = " + str(fprimy(x[t], y[t])))

plt.plot(x)
plt.xlabel('time')
plt.ylabel('x')
plt.show()

plt.plot(y)
plt.xlabel('time')
plt.ylabel('y')
plt.show()