#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from shapely import wkt
from rclpy.node import Node
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose,PoseArray
# get share path
from ament_index_python.packages import get_package_share_directory

share_dir_path = get_package_share_directory('dubins_planner')
print(share_dir_path)


class WaypointsPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(
            PoseArray, 'shelfinoG/waypoints', 10)
        print("Node started")

    def get_data(self):
        pts = np.loadtxt(
            share_dir_path+"/data/voronoi_points.csv", skiprows=1, delimiter=',')
        # first point of each edge
        self.x1 = pts[:, 0]
        self.y1 = pts[:, 1]
        # second point of each edge
        self.x2 = pts[:, 2]
        self.y2 = pts[:, 3]

    def plot(self):
        self.get_data()
        _, ax = plt.subplots()
        fig = wkt.loads(
            open(share_dir_path+"/data/multipolygon_data.txt").read())
        for geom in fig.geoms:
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
        plt.show()

    def publish(self):
        self.get_data()
        # Since x2[i],y2[i] = x1[i+1],y1[i+1] we can use just x1,y1
        msg_array = PoseArray()
        for i in range(len(self.x1)):
            msg = Pose()
            msg.position.x = self.x1[i]
            msg.position.y = self.y1[i]
            msg.position.z = 0.0
            msg.orientation.w = 1.0
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            # Add the message to the array
            msg_array.poses.append(msg)
        # Publish the array
        self.publisher_.publish(msg_array)


def main(args=None):
    rclpy.init(args=args)

    node = WaypointsPublisher()
    node.plot()
    node.publish()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
