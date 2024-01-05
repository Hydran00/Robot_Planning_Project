#! /usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

def draw_point(point, arrow_length=0.5):
    plt.plot(point[0], point[1], 'o')
    # plt.arrow(point[0], point[1], arrow_length * np.math.cos(point[2]), arrow_length * np.sin(point[2]), head_width=0.05)

def test2(path):
    start = path[0]
    end = path[-1]
    
    plt.figure()
    # for i in range(12):
    plt.title('{}{}{}'.format(path[0][0], path[1][0], path[2][0]))
    draw_point(start)
    draw_point(end)
    plt.plot(path[:,0], path[:,1])
    plt.axis("equal")
    plt.show()

# Example shortest path (replace this with the actual shortest path obtained from your C++ code)
example_shortest_path = [['s', 2.0, 0.0, 0.0, 0.0],
                         ['r', 1.0, np.pi / 2, 2.0, 0.0],
                         ['s', 3.0, 0.0, 3.0, 1.0],
                         ['l', 2.0, -np.pi / 3, 0.0, 4.0]]

if __name__ == "__main__":
    path = np.array([
        [-1, -2],
        [-1.47943, -2.12242],
        [-1.84147, -2.4597],
        [-1.99749, -2.92926],
        [-1.9093, -3.41615],
        [-1.59847, -3.80114],
        [-1.14112, -3.98999],
        [-0.649217, -3.93646],
        [-0.243198, -3.65364],
        [-0.080855, -3.39392],
        [-0.080855, -3.39392],
        [0.116105, -2.93435],
        [0.313064, -2.47477],
        [0.510024, -2.0152],
        [0.706984, -1.55563],
        [0.903943, -1.09606],
        [1.1009, -0.636484],
        [1.29786, -0.176912],
        [1.49482, 0.282661],
        [1.69178, 0.742233],
        [1.88874, 1.20181],
        [2.0857, 1.66138],
        [2.28266, 2.12095],
        [2.47962, 2.58052],
        [2.67658, 3.0401],
        [2.87354, 3.49967],
        [2.91915, 3.60608],
        [2.91915, 3.60608],
        [3, 4]])

    test2(path)
