import numpy as np
import matplotlib.pyplot as plt
from math import *
# X = np.arange(0, 5, 0.1)
# Z = [3 + 5 * x for x in X]
# Y = [np.random.normal(z, 0.5) for z in Z]
# plt.plot(X, Y, 'ro')
# plt.show()

def sampling_points_visualization(x,y):
    plt.plot(x,y,'ro')
    plt.show()
def linear_regression(x,y):
    N = len(x)
    sumx = sum(x)
    sumy = sum(y)
    sumx2 = sum(x ** 2)
    sumxy = sum(x * y)
    A = np.mat([[N, sumx], [sumx, sumx2]])
    b = np.array([sumy, sumxy])
    return np.linalg.solve(A, b)

def visualization(x,y,a0,a1):
    _x=[int(x[0]), int(x[len(x)-1])]
    _y=[a0+a1*i for i in _x]
    plt.plot(x,y,'ro')
    plt.plot(_x,_y,'b',linewidth=2)
    plt.show()
def main():
    x=np.array([43777.0, 43809.0, 43821.0, 43839.0, 43856.0, 43876.0])
    y=np.array([90-89.05, 90-76.95, 90-72.55, 90-65.85, 90-59.60, 90-52.35])
    for i in range(len(y)):
        y[i]=y[i]*pi/180.0
    # sampling_points_visualization(x,y)
    a0,a1 = linear_regression(x,y)
    print("a0 is:",a0)
    print("a1 is:",a1)
    print("homing encode data is:",-a0/a1)
    visualization(x,y,a0,a1)


    # ('a0 is:', -283.19582738776523)
    # ('a1 is:', 0.006469488185120742)
    # ('homing encode data is:', 43774.069800311394)


if __name__ == "__main__":
    main()


