#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

class DroneOdometryPublisher(Node):
    def __init__(self):
        super().__init__('drone_odometry_publisher')
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Subscribers for ground truth from Gazebo (if available)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/model/drone_frame/pose',
            self.pose_callback,
            10
        )
        
        # Timer for odometry publishing
        self.timer = self.create_timer(0.1, self.publish_odometry)  # 10 Hz
        
        # State variables
        self.last_pose = None
        self.last_time = None
        self.current_twist = TwistStamped()
        
        self.get_logger().info('Drone Odometry Publisher initialized')
        
    def pose_callback(self, msg):
        """Process pose updates from Gazebo"""
        current_time = self.get_clock().now()
        
        if self.last_pose is not None and self.last_time is not None:
            # Calculate time difference
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            if dt > 0:
                # Calculate linear velocities
                dx = msg.pose.position.x - self.last_pose.pose.position.x
                dy = msg.pose.position.y - self.last_pose.pose.position.y
                dz = msg.pose.position.z - self.last_pose.pose.position.z
                
                self.current_twist.twist.linear.x = dx / dt
                self.current_twist.twist.linear.y = dy / dt
                self.current_twist.twist.linear.z = dz / dt
                
                # Calculate angular velocities (simplified)
                # Convert quaternions to euler angles
                current_euler = euler_from_quaternion([
                    msg.pose.orientation.x, msg.pose.orientation.y,
                    msg.pose.orientation.z, msg.pose.orientation.w
                ])
                
                last_euler = euler_from_quaternion([
                    self.last_pose.pose.orientation.x, self.last_pose.pose.orientation.y,
                    self.last_pose.pose.orientation.z, self.last_pose.pose.orientation.w
                ])
                
                # Calculate angular velocity differences
                droll = self.angle_diff(current_euler[0], last_euler[0])
                dpitch = self.angle_diff(current_euler[1], last_euler[1])
                dyaw = self.angle_diff(current_euler[2], last_euler[2])
                
                self.current_twist.twist.angular.x = droll / dt
                self.current_twist.twist.angular.y = dpitch / dt
                self.current_twist.twist.angular.z = dyaw / dt
        
        # Update stored values
        self.last_pose = msg
        self.last_time = current_time
        
    def angle_diff(self, a, b):
        """Calculate the shortest angular difference between two angles"""
        diff = a - b
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff
        
    def publish_odometry(self):
        """Publish odometry message"""
        if self.last_pose is None:
            return
            
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'
        
        # Copy pose
        odom.pose.pose = self.last_pose.pose
        
        # Copy twist
        odom.twist.twist = self.current_twist.twist
        
        # Set covariance (simplified - identity matrices)
        odom.pose.covariance = [0.1] * 36
        odom.twist.covariance = [0.1] * 36
        
        # Publish
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    publisher = DroneOdometryPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
