#!/usr/bin/env python3

import math
import matplotlib.pyplot as plt
import numpy as np
from shapely import wkt
from rclpy.node import Node
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose,PoseArray
# get share path
from ament_index_python.packages import get_package_share_directory
import networkx as nx
import random as rand
import time as tm
import tf2_ros




def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def compute_yaw_angle(point1, point2):
    # Calculate the differences in x, y coordinates
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]

    # Calculate the yaw angle using arctan2
    yaw_angle = math.atan2(dy, dx)

    # Convert radians to degrees
    # yaw_angle_deg = math.degrees(yaw_angle)

    return yaw_angle

class WaypointsPublisher(Node):
    def __init__(self,resource_path):
        super().__init__('waypoint_publisher')
        self.voronoi_data_path = resource_path+"/data/boost_voronoi_edges.csv"
        self.map_data_path = resource_path+"/data/polygon.txt"
        self.publisher_ = self.create_publisher(PoseArray, 'voronoi_waypoints', 10)
        
        self.get_logger().info('Node started')


    def create_graph(self, x1, y1, x2, y2):
        G = nx.Graph()
        for i in range(len(x1)):
            if(not G.has_node((x1[i], y1[i]))):
                G.add_node((x1[i], y1[i]))
            # euclidean  distance
            G.add_edge((x1[i], y1[i]), (x2[i], y2[i]))
            G[(x1[i], y1[i])][(x2[i], y2[i])]['weight'] = np.sqrt((x1[i]-x2[i])**2+(y1[i]-y2[i])**2)
        self.G = G
        print("Voronoi graph created with the following nodes:\n",list(G.nodes))

        self.shortest_path = list(nx.shortest_path(self.G, source=list(G.nodes)[0], target=list(G.nodes)[-1]))
        print("#####################################")
        print("Shortest path: \n", self.shortest_path)


    def get_data(self):
        pts = np.loadtxt(self.voronoi_data_path, skiprows=1, delimiter=',')
        # first point of each edge
        self.x1 = pts[:, 0]
        self.y1 = pts[:, 1]
        # second point of each edge
        self.x2 = pts[:, 2]
        self.y2 = pts[:, 3]
        self.create_graph(self.x1, self.y1, self.x2, self.y2)
        self.plot()

    def plot(self):
        _, ax = plt.subplots()
        geom = wkt.loads(open(self.map_data_path).read())
        # for geom in fig.geoms:
            # plot the exterior polygons
        xs, ys = geom.exterior.xy
        ax.plot(xs, ys, '-ok', lw=4)
        for hole in geom.interiors:
            # plot holes for each polygon
            xh, yh = hole.xy
            ax.plot(xh, yh, '-ok', lw=4)
            # plot the voronoi edges
        ax.plot((self.x1, self.x2), (self.y1, self.y2), '-o')
        # ax.plot(self.x1, self.y1, '-o')
        # ax.plot(0, 0, 'ro')

        # plot shortest path
        plt.plot([x[0] for x in self.shortest_path], [x[1] for x in self.shortest_path], '-bo')
        plt.axis('equal')
        plt.show(block=False)

    def publish(self):
        self.get_data()
        # Since x2[i],y2[i] = x1[i+1],y1[i+1] we can use just x1,y1
        # msg_array = PoseArray()
        # msg_array.header.frame_id = "map"
        # msg_array.header.stamp = self.get_clock().now().to_msg()
        # for i in range(len(self.shortest_path)):
        #     print("Waypoint ", i, ": (", self.shortest_path[i][0]," - ",self.shortest_path[i][1],")")
        #     # compute orintation
        #     if(i == len(self.shortest_path)-1):
        #         yaw = compute_yaw_angle(self.shortest_path[i-1], self.shortest_path[i])
        #     else:
        #         yaw = compute_yaw_angle(self.shortest_path[i], self.shortest_path[i+1])
        #     quaternion = quaternion_from_euler(0, 0, yaw)

        #     self.get_logger().info("x: {}, y: {}, yaw: {}".format(self.shortest_path[i][0], self.shortest_path[i][1], yaw))
        #     # convert yaw to quaternion

        #     msg = Pose()
        #     msg.position.x = self.shortest_path[i][0]
        #     msg.position.y = self.shortest_path[i][1]
        #     msg.position.z = 0.0
        #     msg.orientation.x = quaternion[0]
        #     msg.orientation.y = quaternion[1]
        #     msg.orientation.z = quaternion[2]
        #     msg.orientation.w = quaternion[3]
        #     msg_array.poses.append(msg)
        #     # Add the message to the array
        # # Publish the array
        # print("Publishing msg ",msg_array)
        # for i in range(50):
        #     self.publisher_.publish(msg_array)
        #     tm.sleep(0.1)
        plt.show()
        exit()

def plot_just_map():
    _, ax = plt.subplots()
    geom = wkt.loads("POLYGON((-5 8.66,5 8.66,10 0,5 -8.66,-5 -8.66,-10 0,-5 8.66,-5 8.66),(-3.57639 -0.710994,-3.57639 1.70375,-1.59629 1.70375,-1.59629 -0.710994,-3.57639 -0.710994),(0.0184117 2.0726,0.0184117 3.76496,1.38452 3.76496,1.38452 2.0726,0.0184117 2.0726),(1.10357 4.91946,1.10357 7.09519,2.43253 7.09519,2.43253 4.91946,1.10357 4.91946),(-1.64867 -6.10781,-1.64867 -4.17029,-0.336099 -4.17029,-0.336099 -6.10781,-1.64867 -6.10781),(5.62716 2.90627,5.62716 4.46288,7.92721 4.46288,7.92721 2.90627,5.62716 2.90627),(4.83935 -5.96126,4.83935 -4.06067,6.33404 -4.06067,6.33404 -5.96126,4.83935 -5.96126),(2.55184 2.60307,2.55184 4.29577,4.86508 4.29577,4.86508 2.60307,2.55184 2.60307),(-4.99154 -6.94937,-4.99154 -5.04944,-3.37838 -5.04944,-3.37838 -6.94937,-4.99154 -6.94937),(0.329475 -3.96557,0.329475 -2.37938,2.50814 -2.37938,2.50814 -3.96557,0.329475 -3.96557),(1.22275 -1.16595,1.22275 1.29612,2.57393 1.29612,2.57393 -1.16595,1.22275 -1.16595),(-4.30773 2.6883,-4.30773 4.41351,-2.83002 4.41351,-2.83002 2.6883,-4.30773 2.6883),(-6.94342 -1.71789,-6.94342 0.40597,-5.45672 0.40597,-5.45672 -1.71789,-6.94342 -1.71789),(-4.65277 -3.47797,-4.65277 -1.62802,-2.18907 -1.62802,-2.18907 -3.47797,-4.65277 -3.47797),(-6.91829 2.95523,-6.91829 4.63563,-5.07439 4.63563,-5.07439 2.95523,-6.91829 2.95523),(5.00636 0.610004,5.00636 2.74982,6.35819 2.74982,6.35819 0.610004,5.00636 0.610004),(-1.64574 5.23002,-1.64574 7.03132,0.568999 7.03132,0.568999 5.23002,-1.64574 5.23002),(2.26245 -7.04564,2.26245 -5.31188,4.44484 -5.31188,4.44484 -7.04564,2.26245 -7.04564),(4.58881 4.56598,4.58881 6.03924,6.50009 6.03924,6.50009 4.56598,4.58881 4.56598),(-8.2888 0.868923,-8.2888 2.81826,-6.04971 2.81826,-6.04971 0.868923,-8.2888 0.868923),(2.7253 -2.45245,2.7253 -0.191882,4.24435 -0.191882,4.24435 -2.45245,2.7253 -2.45245))")

    path = wkt.loads("LINESTRING(0.33838 0.184357,0.35509 0.23146,0.367013 0.279996,0.374032 0.32948,0.376075 0.379418,0.373123 0.42931,0.365204 0.478657,0.352399 0.526968,0.334834 0.573759,0.312686 0.618563,0.286175 0.660932,0.255568 0.700442,0.221168 0.7367,0.183321 0.769342,0.142404 0.798043,0.0988265 0.822515,0.0530233 0.842515,0.00545229 0.857842,-0.0434112 0.868343,-0.093079 0.873913,-0.143055 0.874498,-0.192839 0.87009,-0.241935 0.860734,-0.289851 0.846523,-0.336109 0.8276,-0.380247 0.804153,-0.421824 0.776416,-0.460424 0.744668,-0.495661 0.709225,-0.527184 0.67044,-0.554678 0.628703,-0.577868 0.584429,-0.596521 0.538062,-0.610453 0.490063,-0.619523 0.440914,-0.623641 0.391105,-0.623582 0.356921,-0.623582 0.356921,-0.605639 -0.142757,-0.587695 -0.642435,-0.569751 -1.14211,-0.551807 -1.64179,-0.541361 -1.93268,-0.541361 -1.93268,-0.542066 -1.98265,-0.547757 -2.03231,-0.558376 -2.08114,-0.573818 -2.12868,-0.593928 -2.17443,-0.618505 -2.21795,-0.647305 -2.2588,-0.680038 -2.29657,-0.716379 -2.33088,-0.755963 -2.36139,-0.798396 -2.3878,-0.843253 -2.40984,-0.890086 -2.42729,-0.938428 -2.43998,-0.987794 -2.44778,-1.03769 -2.45061,-1.08763 -2.44845,-1.13709 -2.44131,-1.1856 -2.42927,-1.23266 -2.41244,-1.27781 -2.39101,-1.32059 -2.36517,-1.36058 -2.33519,-1.39738 -2.30137,-1.43061 -2.26404,-1.45996 -2.22358,-1.48512 -2.1804,-1.50584 -2.13491,-1.52191 -2.08759,-1.53318 -2.0389,-1.53954 -1.98933,-1.54091 -1.93937,-1.53872 -1.90251)")
    xs, ys = geom.exterior.xy
    ax.plot(xs, ys, '-ok', lw=4)
    for hole in geom.interiors:
        # plot holes for each polygon
        xh, yh = hole.xy
        ax.plot(xh, yh, '-ok', lw=4)
        # plot the voronoi edges
    # [-0.619676, -1.472391] and [-1.341120, -1.791085]
    # ax.plot([-0.619676, -1.341120], [-1.472391, -1.791085], '-o')
    #plot path
    xs, ys = path.xy
    ax.plot(xs, ys, '-o')
    
    plt.show()
    exit(0)

def main(args=None):
    plot_just_map()
    rclpy.init(args=args)
    
    share_dir_path = get_package_share_directory('planner')
    node = WaypointsPublisher(resource_path=share_dir_path)
    node.publish()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
