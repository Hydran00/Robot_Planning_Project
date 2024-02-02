#! /usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
from ament_index_python.packages import get_package_share_directory
def draw_point(point, arrow_length=0.5):
    plt.plot(point[0], point[1], 'o')
    # plt.arrow(point[0], point[1], arrow_length * np.math.cos(point[2]), arrow_length * np.sin(point[2]), head_width=0.05)

def distance(a,b):
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    
def test2(path):
    start = path[0]
    end = path[-1]
    plt.figure()
    # for i in range(12):
    plt.title("Dubins best path")
    draw_point(start)
    draw_point(end)
    plt.plot(path[:,0], path[:,1], 'ok-')
    plt.axis("equal")
    plt.show()


def test3(path1, path2):
    plt.figure()

    start = path1[0]
    end = path1[-1]
    plt.title("Dubins best path")
    draw_point(start)
    draw_point(end)
    plt.plot(path1[:,0], path1[:,1], 'ok-')

    start = path2[0]
    end = path2[-1]
    draw_point(start)
    draw_point(end)
    plt.plot(path2[:,0], path2[:,1], 'or-', linewidth=0.5, markersize=0.5)
    
    tot_dist1 = 0
    tot_dist2 = 0
    for i in range(1,len(path1),1):
        tot_dist1 += distance(path1[i], path1[i-1])
    for i in range(1,len(path2),1):
        tot_dist2 += distance(path2[i], path2[i-1])
    print("Total distance for path: \t\t", tot_dist1)
    print("Total distance for opt path: \t\t    ", tot_dist2)
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    # path1 = np.loadtxt(str(get_package_share_directory('planner')) + '/data/final_path0.txt', delimiter=',')
    path2 = np.loadtxt(str(get_package_share_directory('planner')) + '/data/final_path.txt', delimiter=',')
    # def distance(a,b):
    #     return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    # for i in range(1,len(path),1):
    #     if(distance(path[i], path[i-1]) > 0.15):
    #         print("Inconstency at index ",i)
    # test2(path1)
    test2(path2)
    # test3(path1, path2)
