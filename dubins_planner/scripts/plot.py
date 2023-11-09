#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from shapely import wkt

fig, ax = plt.subplots()

# Output file from C++ medial axis code
pts = np.loadtxt("dubins_planner/data/voronoi_points.csv", skiprows=1, delimiter=',')
x1=pts[:,0]
y1=pts[:,1]
x2=pts[:,2]
y2=pts[:,3]
fig = wkt.loads(open("dubins_planner/data/multipolygon_data.txt").read())
for geom in fig.geoms:
  xs, ys = geom.exterior.xy
  for hole in geom.interiors:
    xh, yh = hole.xy
    ax.plot(xh, yh, '-ok', lw=4)
  ax.plot(xs, ys, '-ok', lw=4)

# for i in range(len(x1)):
#   ax.plot((x1[i],y1[i]),(x2[i],y2[i]), '-o')
ax.plot( (x1,x2),(y1,y2), '-o')
# # Output file from C++ medial axis code
# ma = np.loadtxt("build/voronoi_edges.csv", dtype=int, skiprows=1, delimiter=',')
# for mal in ma:
#   print(mal)
#   ax.plot((pts[mal[0]][0], pts[mal[1]][0]), (pts[mal[0]][1], pts[mal[1]][1]), '-o')
ax.plot(0,0, 'ro')
plt.show()
