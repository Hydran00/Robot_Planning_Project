# Robot Planning Projects
We implemented various variants of RRT and the Voronoi shortest path for
the victim rescuing. The repository available on [github](https://github.com/Hydran00/Robot_Planning_Project/tree/main)

### How to test it
```
ros2 launch projects victims.launch.py
```
and then 
```
ros2 launch planner <planner_name>.launch.py
```
finaly the missing nav2 client
```
ros2 run planner nav2_client
```
Available planners are:
  - RRT (``rrt``)
  - RRT* with our victim rescuing algorithm (``rrt_star``)
  - RRT* Dubins with our victim rescuing algorithm  (``rrt_star_dubins``)
  - Multithreaded version of rrt* and rrt*_dubins (``multithread_planner.launch``)
  - Voronoi with TSP brute force solution (``voronoi_planner``)