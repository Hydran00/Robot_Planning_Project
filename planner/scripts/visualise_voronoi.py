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

def plot_map():
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
def plot_voronoi(data):
    x1 = data[:,0]
    y1 = data[:,1]
    x2 = data[:,2]
    y2 = data[:,3]
    plt.plot([x1, x2], [y1, y2], 'k-')

def plot_path(data):    
    # plot path
    x = data[:,0]
    y = data[:,1]
    plt.plot(x, y, 'ro--')
    
if __name__ == "__main__":
    plot_map()
    if os.path.exists(str(get_package_share_directory('planner')) + '/data/voronoi.txt'):
        voronoi = np.loadtxt(str(get_package_share_directory('planner')) + '/data/voronoi.txt')
        plot_voronoi(voronoi)
    if os.path.exists(str(get_package_share_directory('planner')) + '/data/voronoi_path.txt'):
        voronoi_path = np.loadtxt(str(get_package_share_directory('planner')) + '/data/voronoi_path.txt')
        plot_path(voronoi_path)
    elif os.path.exists(str(get_package_share_directory('planner')) + '/data/final_path.txt'):
        dubins_path = np.loadtxt(str(get_package_share_directory('planner')) + '/data/final_path.txt')
        plot_path(dubins_path)
    else:
        print("No path found")
    plt.show()
    

