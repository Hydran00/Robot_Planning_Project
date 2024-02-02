#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import ColorRGBA

from std_msgs.msg import Header

class TextMarkerPublisher(Node):
    def __init__(self):
        super().__init__('text_marker_publisher_node')
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_text_marker)
        self.text_marker_id = 0

    def publish_text_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Update with your desired frame ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = self.text_marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(x=1.0, y=1.0, z=0.0)  # Set the position as per your requirement
        marker.pose.orientation = Pose().orientation
        marker.scale = Vector3(x=0.5, y=0.5, z=0.5)  # Set the scale as per your requirement
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red color
        marker.text = "Hello, RViz Marker!"
        print("Publishing ", marker)
        self.marker_publisher.publish(marker)
        self.text_marker_id += 1

def main(args=None):
    rclpy.init(args=args)
    text_marker_publisher = TextMarkerPublisher()
    rclpy.spin(text_marker_publisher)
    text_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

