#! /usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
from ament_index_python.packages import get_package_share_directory
from shapely import wkt
import os


def plot2(data):
    plt.axis('equal')
    # plot voronoi
    x1 = data[:,0]
    y1 = data[:,1]
    x2 = data[:,2]
    y2 = data[:,3]
    plt.plot([x1, x2], [y1, y2], 'k-')
    plt.show()
    plt.show(block=False)

def plot_map(data1,data2):
    _, ax = plt.subplots()
    plt.axis('equal')
    geom = wkt.loads(open(get_package_share_directory('planner') + '/data/map.txt').read())

    # for geom in fig.geoms:
        # plot the exterior polygons
    xs, ys = geom.exterior.xy
    ax.plot(xs, ys, '-ob', lw=6)
    for hole in geom.interiors:
        # plot holes for each polygon
        xh, yh = hole.xy
        ax.plot(xh, yh, '-ob', lw=6)
        # plot the voronoi edges

    # plot voronoi
    x1 = data[:,0]
    y1 = data[:,1]
    x2 = data[:,2]
    y2 = data[:,3]
    plt.plot([x1, x2], [y1, y2], 'k-')
    
    # plot path
    x = data2[:,0]
    y = data2[:,1]

    plt.plot(x, y, 'ro--')
    plt.show()
    
if __name__ == "__main__":
    # path1 = np.loadtxt(str(get_package_share_directory('planner')) + '/data/final_path.txt', delimiter=',')
    data = np.loadtxt(str(get_package_share_directory('planner')) + '/data/voronoi.txt')
    # plot_map(data)   

    if os.path.exists(str(get_package_share_directory('planner')) + '/data/best_path_voronoi.txt'):
        data2 = np.loadtxt(str(get_package_share_directory('planner')) + '/data/best_path_voronoi.txt')
        plot_map(data,data2) 
    else:
        plot2(data)
    # data2 = np.loadtxt("/home/hydran00/shelfino_ws/install/planner/share/planner/data/scaled_poly.txt")
    # plot2(data2)
