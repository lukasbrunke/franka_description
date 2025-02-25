#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import numpy as np
import pinocchio as pin

class MultiMarkerPublisher(Node):
    def __init__(self, shapes):
        super().__init__('multi_marker_publisher')

        self.shapes = shapes
        
        # Create publishers for each marker type
        self.marker_publishers = {}
        for key in self.shapes:
            self.marker_publishers[key] = self.create_publisher(Marker, f'visualization_marker_{key}', 10)
        # self.marker_publishers = {
        #     'ellipsoid': self.create_publisher(Marker, 'visualization_marker_ellipsoid', 10),
        #     'cube': self.create_publisher(Marker, 'visualization_marker_cube', 10),
        #     'cylinder': self.create_publisher(Marker, 'visualization_marker_cylinder', 10),
        #     'arrow': self.create_publisher(Marker, 'visualization_marker_arrow', 10)
        # }
        
        # Also create a publisher for the standard visualization_marker topic
        # This allows showing all markers in one RViz display
        self.common_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # Set up a timer to publish all markers at 10Hz
        self.timer = self.create_timer(1.0 / 60.0, self.publish_all_markers)
        
        self.get_logger().info('Multi marker publisher started')
        
    def publish_all_markers(self):
        # self.publish_ellipsoid()
        # self.publish_cube()
        # self.publish_cylinder()
        # self.publish_arrow()
        for key in self.shapes:
            if self.shapes[key]['type'] == 'superquadric':
                self.publish_superquadric(key)
            else:
                self.publish_shape(key)

    def publish_shape(self, key):
        shape = self.shapes[key]
        marker = Marker()
        marker.header.frame_id = shape['frame_id']
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = int(key)  # Unique ID
        if shape['type'] == 'cube':
            marker.type = Marker.CUBE
        elif shape['type'] == 'cylinder':
            marker.type = Marker.CYLINDER
        elif shape['type'] == 'arrow':
            marker.type = Marker.ARROW
        elif shape['type'] == 'ellipsoid':
            marker.type = Marker.SPHERE
        else:
            raise ValueError(f"Invalid shape type: {shape['type']}")
        # marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = shape['position'][0]
        marker.pose.position.y = shape['position'][1]
        marker.pose.position.z = shape['position'][2]
        
        # Set rotation from axis angle
        quat = pin.Quaternion(pin.exp3(np.array(shape['rotation'])))
        quat.normalize()
        quat = quat.coeffs()
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        
        # Different scale for each dimension
        marker.scale.x = shape['scale'][0]  # in x-direction
        marker.scale.y = shape['scale'][1]  # in y-direction
        marker.scale.z = shape['scale'][2]  # in z-direction
        
        # Color (blue)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8  # Slightly transparent
        
        # Set the lifetime (0 means forever)
        marker.lifetime.sec = 0
        
        # Publish to the type-specific topic
        self.marker_publishers[key].publish(marker)
        # Also publish to the common topic
        self.common_publisher.publish(marker)
    
    def publish_ellipsoid(self):
        marker = Marker()
        marker.header.frame_id = "fr3_link0"  # Use the robot's base frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0  # Unique ID
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = 0.5
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        
        # No rotation
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Different scale for each dimension creates an ellipsoid
        marker.scale.x = 0.2  # 20cm in x-direction
        marker.scale.y = 0.1  # 10cm in y-direction
        marker.scale.z = 0.15  # 15cm in z-direction
        
        # Color (blue)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8  # Slightly transparent
        
        # Set the lifetime (0 means forever)
        marker.lifetime.sec = 0
        
        # Publish to the type-specific topic
        self.marker_publishers['ellipsoid'].publish(marker)
        # Also publish to the common topic
        self.common_publisher.publish(marker)
    
    def publish_cube(self):
        marker = Marker()
        marker.header.frame_id = "fr3_link7"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 1  # Different ID
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.127
        
        # Set rotation from axis-angle
        axis_angle = np.array([0.0, 0.0, np.pi/4])
        quat = pin.Quaternion(pin.exp3(axis_angle))
        quat.normalize()
        quat = quat.coeffs()
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        
        # Scale
        marker.scale.x = 0.22
        marker.scale.y = 0.04
        marker.scale.z = 0.095
        
        # Color (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        
        marker.lifetime.sec = 0
        self.marker_publishers['cube'].publish(marker)
        self.common_publisher.publish(marker)

    def publish_cylinder(self):
        marker = Marker()
        marker.header.frame_id = "fr3_link5"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 2  # Different ID
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.09
        marker.pose.position.z = -0.1
        
        # Set rotation from axis-angle
        axis_angle = np.array([np.pi/2, 0.0, 0.0])
        T_x1 = pin.SE3(pin.exp3(axis_angle), np.array([0.0, 0.0, 0.0]))
        axis_angle = np.array([-10.0 / 180.0 * np.pi, 0.0, 0.0])
        T_x2 = pin.SE3(pin.exp3(axis_angle), np.array([0.0, 0.0, 0.0]))
        T = T_x2 * T_x1
        R = T.rotation
        axis_angle = pin.log3(R).flatten()
        quat = pin.Quaternion(pin.exp3(axis_angle))
        quat.normalize()
        quat = quat.coeffs()
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        
        # Scale
        marker.scale.x = 0.08  # diameter in x
        marker.scale.y = 0.285  # diameter in y
        marker.scale.z = 0.05  # height in z
        
        # Color (green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        
        marker.lifetime.sec = 0
        self.marker_publishers['cylinder'].publish(marker)
        self.common_publisher.publish(marker)
    
    def publish_arrow(self):
        marker = Marker()
        marker.header.frame_id = "fr3_link0"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 3  # Different ID
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = 0.3
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.7
        
        # Point the arrow forward
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Scale for arrows: x is shaft length, y is shaft diameter, z is head diameter
        marker.scale.x = 0.2  # arrow length
        marker.scale.y = 0.02  # shaft diameter
        marker.scale.z = 0.05  # head diameter
        
        # Color (yellow)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 0
        self.marker_publishers['arrow'].publish(marker)
        self.common_publisher.publish(marker)

    def publish_superquadric(self, key):
        shape = self.shapes[key]
        # Get current parameters
        a1 = shape['scale'][0]
        a2 = shape['scale'][1]
        a3 = shape['scale'][2]
        e1 = shape['exponent'][0]
        e2 = shape['exponent'][1]
        pos_x = shape['position'][0]
        pos_y = shape['position'][1]
        pos_z = shape['position'][2]
        
        # Generate the superquadric mesh
        vertices, triangles = generate_superquadric_mesh(a1, a2, a3, e1, e2)
        
        # Create a marker for the mesh
        marker = Marker()
        marker.header.frame_id = shape['frame_id']
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = pos_x
        marker.pose.position.y = pos_y
        marker.pose.position.z = pos_z
        
        # Set rotation from axis angle
        quat = pin.Quaternion(pin.exp3(np.array(shape['rotation'])))
        quat.normalize()
        quat = quat.coeffs()
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        
        # Scale - for TRIANGLE_LIST this is just 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # Color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        # Add all triangles to the marker
        for triangle in triangles:
            for vertex_idx in triangle:
                p = Point()
                p.x = vertices[vertex_idx][0]
                p.y = vertices[vertex_idx][1]
                p.z = vertices[vertex_idx][2]
                marker.points.append(p)

        # Set the lifetime (0 means forever)
        marker.lifetime.sec = 0
        
        # Publish to the type-specific topic
        self.marker_publishers[key].publish(marker)
        # Also publish to the common topic
        self.common_publisher.publish(marker)


def generate_superquadric_mesh(a1, a2, a3, e1, e2, resolution=20):
    """
    Generate a superquadric mesh.
    
    Parameters:
    a1, a2, a3: scale parameters along x, y, and z axes
    e1: east-west exponent
    e2: north-south exponent
    resolution: number of points along each parameter
    
    Returns:
    vertices: list of 3D points
    triangles: list of triangle indices
    """
    # Generate parameter space
    eta = np.linspace(-np.pi/2, np.pi/2, resolution)
    omega = np.linspace(-np.pi, np.pi, resolution)
    eta, omega = np.meshgrid(eta, omega)
    
    # Compute the surface
    x = a1 * sign_pow(np.cos(eta), e1) * sign_pow(np.cos(omega), e2)
    y = a2 * sign_pow(np.cos(eta), e1) * sign_pow(np.sin(omega), e2)
    z = a3 * sign_pow(np.sin(eta), e1)
    
    # Create vertices and faces for the mesh
    vertices = []
    for i in range(resolution):
        for j in range(resolution):
            vertices.append([x[i, j], y[i, j], z[i, j]])
    
    triangles = []
    for i in range(resolution-1):
        for j in range(resolution-1):
            # Calculate vertex indices
            v0 = i * resolution + j
            v1 = i * resolution + (j + 1)
            v2 = (i + 1) * resolution + j
            v3 = (i + 1) * resolution + (j + 1)
            
            # Add two triangles for each quad face
            triangles.append([v0, v1, v2])
            triangles.append([v1, v3, v2])
    
    return vertices, triangles
    

def sign_pow(x, a):
    """
    Compute sign(x) * |x|^a
    """
    return np.sign(x) * (np.abs(x) ** a)


def main(args=None):
    # End effector shape orientation
    axis_angle = np.array([0.0, 0.0, np.pi/4])
    T_z = pin.SE3(pin.exp3(axis_angle), np.array([0.0, 0.0, 0.0]))
    axis_angle = np.array([np.pi/2, 0.0, 0.0])
    T_y = pin.SE3(pin.exp3(axis_angle), np.array([0.0, 0.0, 0.0]))
    T = T_z * T_y
    R = T.rotation
    axis_angle_ee = pin.log3(R).flatten()

    # Link 5 shape orientation
    axis_angle_ = np.array([np.pi/2, 0.0, 0.0])
    T_x1 = pin.SE3(pin.exp3(axis_angle_), np.array([0.0, 0.0, 0.0]))
    axis_angle_ = np.array([-10.0 / 180.0 * np.pi, 0.0, 0.0])
    T_x2 = pin.SE3(pin.exp3(axis_angle_), np.array([0.0, 0.0, 0.0]))
    T = T_x2 * T_x1
    R = T.rotation
    axis_angle_link5 = pin.log3(R).flatten()

    # Define the shapes to be published
    shapes = {
        0 : {
            'type': 'superquadric',
            'frame_id': 'fr3_link7',
            'position': [0.0, 0.0, 0.13],
            'rotation': axis_angle_ee,  # axis-angle
            'scale': [0.11, 0.05, 0.02],
            'exponent': [0.3, 0.3]
        },
        1 : {
            'type': 'superquadric',
            'frame_id': 'fr3_link6',
            'position': [0.0, 0.0, -0.04],
            'rotation': [0.0, 0.0, 0.0],  # axis-angle
            'scale': [0.06, 0.06, 0.1],
            'exponent': [0.5, 1.0]
        },
        2: {
            'type': 'superquadric',
            'frame_id': 'fr3_link6',
            'position': [0.088, -0.016, 0.0],
            'rotation': [np.pi/2, 0.0, 0.0],  # axis-angle
            'scale': [0.05, 0.05, 0.11],
            'exponent': [0.5, 1.0]
        },
        3: {
            'type': 'superquadric',
            'frame_id': 'fr3_link5',
            'position': [0.0, 0.09, -0.1],
            'rotation': axis_angle_link5,  # axis-angle
            'scale': [0.04, 0.15, 0.025],
            'exponent': [0.5, 1.0]
        },
        4: {
            'type': 'superquadric',
            'frame_id': 'fr3_link4',
            'position': [-0.084, 0.115, 0.0],
            'rotation': [np.pi/2, 0.0, 0.0],  # axis-angle
            'scale': [0.06, 0.06, 0.11],
            'exponent': [0.5, 1.0]
        },
        5: {
            'type': 'superquadric',
            'frame_id': 'fr3_link3',
            'position': [0.088, 0.00, 0.0],
            'rotation': [np.pi/2, 0.0, 0.0],  # axis-angle
            'scale': [0.065, 0.065, 0.12],
            'exponent': [0.5, 1.0]
        },
        6: {
            'type': 'superquadric',
            'frame_id': 'fr3_link2',
            'position': [0.0, -0.2, 0.0],
            'rotation': [np.pi/2, 0.0, 0.0],  # axis-angle
            'scale': [0.065, 0.065, 0.115],
            'exponent': [0.5, 1.0]
        },
        7: {
            'type': 'superquadric',
            'frame_id': 'fr3_link1',
            'position': [0.0, 0.0, 0.0],
            'rotation': [np.pi/2, 0.0, 0.0],  # axis-angle
            'scale': [0.065, 0.065, 0.13],
            'exponent': [0.5, 1.0]
        },
        8 : {
            'type': 'superquadric',
            'frame_id': 'fr3_link7',
            'position': [0.02, 0.02, 0.09],
            'rotation': axis_angle_ee,  # axis-angle
            'scale': [0.07, 0.03, 0.045],
            'exponent': [0.5, 1.0]
        },
    }

    rclpy.init(args=args)
    
    # Create a single node
    publisher = MultiMarkerPublisher(shapes)
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()