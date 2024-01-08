#! /usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
from ament_index_python.packages import get_package_share_directory
def draw_point(point, arrow_length=0.5):
    plt.plot(point[0], point[1], 'o')
    # plt.arrow(point[0], point[1], arrow_length * np.math.cos(point[2]), arrow_length * np.sin(point[2]), head_width=0.05)

def test2(path):
    start = path[0]
    end = path[-1]
    plt.figure()
    # for i in range(12):
    plt.title("Dubins best path")
    draw_point(start)
    draw_point(end)
    plt.plot(path[:,0], path[:,1], 'k-')
    plt.axis("equal")
    plt.show()

if __name__ == "__main__":
    path = np.loadtxt(str(get_package_share_directory('planner')) + '/data/dubins_path.txt', delimiter=',')
    print(path)
    test2(path)
