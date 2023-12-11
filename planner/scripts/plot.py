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
        # sleep 1 sec
        
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

        # self.shortest_path = list(nx.shortest_path(self.G, source=list(G.nodes)[0], target=list(G.nodes)[-1]))
        # print("#####################################")
        # print("Shortest path: \n", self.shortest_path)


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
        # plt.plot([x[0] for x in self.shortest_path], [x[1] for x in self.shortest_path], '-bo')
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


def main(args=None):
    rclpy.init(args=args)
    
    share_dir_path = get_package_share_directory('planner')
    node = WaypointsPublisher(resource_path=share_dir_path)
    node.publish()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
