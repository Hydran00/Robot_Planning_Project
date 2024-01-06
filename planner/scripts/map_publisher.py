#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32
from obstacles_msgs.msg import ObstacleMsg, ObstacleArrayMsg
from geometry_msgs.msg import PolygonStamped as PolygonStampedMsg
from std_msgs.msg import Header

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        self.publisher1 = self.create_publisher(ObstacleArrayMsg, '/obstacles', 10)
        self.publisher2 = self.create_publisher(PolygonStampedMsg, '/borders', 10)
        self.timer1 = self.create_timer(1, self.publish_obstacles)
        self.timer2 = self.create_timer(1, self.publish_borders)
        self.obstacle_array_msg = ObstacleArrayMsg()
        self.borders_msg = PolygonStampedMsg()

    def publish_obstacles(self):
        obstacle_msg1 = ObstacleMsg()
        obstacle_msg1.header = Header()
        obstacle_msg1.header.stamp = self.get_clock().now().to_msg()
        obstacle_msg1.header.frame_id = 'map'
        obstacle_msg1.polygon.points = [
            Point32(x=-1.5, y=2.5, z=0.0),
            Point32(x=-0.5, y=2.5, z=0.0),
            Point32(x=-0.5, y=1.5, z=0.0),
            Point32(x=-1.5, y=1.5, z=0.0)
        ]
        obstacle_msg1.radius = 0.0

        obstacle_msg2 = ObstacleMsg()
        obstacle_msg2.header = Header()
        obstacle_msg2.header.stamp = self.get_clock().now().to_msg()
        obstacle_msg2.header.frame_id = 'map'
        obstacle_msg2.polygon.points = [
            Point32(x=-2.0, y=-2.0, z=0.0),
            Point32(x=-1.0, y=-1.0, z=0.0),
            Point32(x=0.0, y=-2.0, z=0.0)
        ]
        obstacle_msg2.radius = 0.0

        obstacle_msg3 = ObstacleMsg()
        obstacle_msg3.header = Header()
        obstacle_msg3.header.stamp = self.get_clock().now().to_msg()
        obstacle_msg3.header.frame_id = 'map'
        obstacle_msg3.polygon.points = [
            Point32(x=2.0, y=-1.0, z=0.0),
            Point32(x=3.0, y=-1.0, z=0.0),
            Point32(x=3.0, y=1.0, z=0.0),
            Point32(x=2.0, y=1.0, z=0.0)
        ]
        obstacle_msg3.radius = 0.0



        self.obstacle_array_msg.header.stamp = self.get_clock().now().to_msg()
        self.obstacle_array_msg.header.frame_id = 'map'
        self.obstacle_array_msg.obstacles = [obstacle_msg1, obstacle_msg2, obstacle_msg3]

        self.publisher1.publish(self.obstacle_array_msg)


    def publish_borders(self):
        polygon_msg = PolygonStampedMsg()
        polygon_msg.header.stamp = self.get_clock().now().to_msg()
        polygon_msg.header.frame_id = 'map'
        polygon_msg.polygon.points = [
            Point32(x=-2.0, y=4.0, z=0.0),
            Point32(x=2.0, y=4.0, z=0.0),
            Point32(x=5.0, y=0.0, z=0.0),
            Point32(x=2.0, y=-4.0, z=0.0),
            Point32(x=-2.0, y=-4.0, z=0.0),
            Point32(x=-5.0, y=0.0, z=0.0)   
        ]

        self.publisher2.publish(polygon_msg)


def main(args=None):
    rclpy.init(args=args)
    obstacle_publisher = ObstaclePublisher()
    rclpy.spin(obstacle_publisher)
    obstacle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
