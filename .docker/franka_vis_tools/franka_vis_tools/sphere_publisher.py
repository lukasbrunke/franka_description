#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class SpherePublisher(Node):
    def __init__(self):
        super().__init__('sphere_publisher')
        
        # Create a publisher for the Marker message
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # Set up a timer to publish the sphere marker at 10Hz
        self.timer = self.create_timer(0.1, self.publish_sphere)
        
        # Log that we've started
        self.get_logger().info('Sphere publisher started')
        
    def publish_sphere(self):
        # Create a marker message
        marker = Marker()
        
        # Set the frame ID - THIS IS CRITICAL - must match a frame in your robot
        marker.header.frame_id = "fr3_link0"  # Use the robot's base frame
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Set a unique ID for this marker
        marker.id = 0
        
        # Set the type to SPHERE
        marker.type = Marker.SPHERE
        
        # Set the action to ADD
        marker.action = Marker.ADD
        
        # Set the pose (position and orientation)
        marker.pose.position.x = 0.5  # 0.5 meters in front of the robot base
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0  # No rotation
        
        # Set the scale - this determines the size of the sphere
        marker.scale.x = 0.1  # 10cm diameter sphere
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Set the color (RGBA)
        marker.color.r = 0.0
        marker.color.g = 1.0  # Green
        marker.color.b = 0.0
        marker.color.a = 0.8  # Slightly transparent
        
        # Set the lifetime (0 means forever)
        marker.lifetime.sec = 0
        
        # Publish the marker
        self.marker_publisher.publish(marker)
        self.get_logger().debug('Published sphere marker')

def main(args=None):
    rclpy.init(args=args)
    sphere_publisher = SpherePublisher()
    
    try:
        rclpy.spin(sphere_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        sphere_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
