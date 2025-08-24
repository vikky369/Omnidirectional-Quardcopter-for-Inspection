#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker, MarkerArray
import math

class ThrustVectorVisualizer(Node):
    def __init__(self):
        super().__init__('thrust_vector_visualizer')
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/thrust_vectors', 10)
        
        # Subscribers for force topics
        self.force_subs = []
        self.thrust_forces = [Vector3()] * 8
        
        for i in range(1, 9):
            sub = self.create_subscription(
                Vector3,
                f'/thruster_{i}/force',
                lambda msg, motor_id=i-1: self.force_callback(msg, motor_id),
                10
            )
            self.force_subs.append(sub)
        
        # Thruster positions (from thruster.xacro)
        self.thruster_positions = [
            [0.2, 0.026, -0.2],   # thruster_1
            [0.2, 0.026, 0.2],    # thruster_2
            [-0.2, 0.026, -0.2],  # thruster_3
            [-0.2, 0.026, 0.2],   # thruster_4
            [-0.2, -0.066, 0.2],  # thruster_5
            [0.2, -0.066, 0.2],   # thruster_6
            [0.2, -0.066, -0.2],  # thruster_7
            [-0.2, -0.066, -0.2]  # thruster_8
        ]
        
        # Timer to publish markers
        self.timer = self.create_timer(0.1, self.publish_markers)
        
        self.get_logger().info('Thrust Vector Visualizer started')
    
    def force_callback(self, msg, motor_id):
        """Store force data for each thruster"""
        self.thrust_forces[motor_id] = msg
    
    def publish_markers(self):
        """Publish thrust vector arrows as markers"""
        marker_array = MarkerArray()
        
        for i in range(8):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "thrust_vectors"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Position at thruster location
            marker.pose.position.x = self.thruster_positions[i][0]
            marker.pose.position.y = self.thruster_positions[i][1]
            marker.pose.position.z = self.thruster_positions[i][2]
            
            # Get force magnitude
            force = self.thrust_forces[i]
            force_magnitude = math.sqrt(force.x**2 + force.y**2 + force.z**2)
            
            if force_magnitude > 0.01:  # Only show if significant force
                # Normalize force vector for direction
                norm_x = force.x / force_magnitude
                norm_y = force.y / force_magnitude
                norm_z = force.z / force_magnitude
                
                # Convert to quaternion (assuming force points in direction)
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                
                # Scale arrow based on force magnitude
                scale_factor = min(force_magnitude / 100.0, 0.5)  # Limit max size
                marker.scale.x = scale_factor  # Arrow length
                marker.scale.y = 0.02  # Arrow width
                marker.scale.z = 0.02  # Arrow height
                
                # Color based on force magnitude (blue to red gradient)
                marker.color.r = min(force_magnitude / 50.0, 1.0)
                marker.color.g = 0.0
                marker.color.b = max(1.0 - force_magnitude / 50.0, 0.0)
                marker.color.a = 0.8
                
                # Set arrow points
                marker.points = []
                # Start point (at thruster)
                start_point = marker.pose.position
                marker.points.append(start_point)
                
                # End point (in direction of force)
                end_point = marker.pose.position
                end_point.x += norm_x * scale_factor
                end_point.y += norm_y * scale_factor
                end_point.z += norm_z * scale_factor
                marker.points.append(end_point)
                
            else:
                # Hide marker if no significant force
                marker.action = Marker.DELETE
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    visualizer = ThrustVectorVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        visualizer.get_logger().info('Shutting down thrust vector visualizer')
    
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
