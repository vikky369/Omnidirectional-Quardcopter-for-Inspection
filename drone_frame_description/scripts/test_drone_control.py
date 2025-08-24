#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import time
import math

class DroneControlTester(Node):
    def __init__(self):
        super().__init__('drone_control_tester')
        
        # Publishers for testing
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/cmd_pose', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/drone/cmd_vel', 10)
        
        # Subscriber for odometry feedback
        self.odom_sub = self.create_subscription(
            Odometry, '/drone/odom', self.odom_callback, 10
        )
        
        # Test timer
        self.test_timer = self.create_timer(2.0, self.run_test_sequence)
        self.test_step = 0
        self.current_odom = None
        
        self.get_logger().info('Drone Control Tester initialized')
        
    def odom_callback(self, msg):
        """Store current odometry for monitoring"""
        self.current_odom = msg
        
    def run_test_sequence(self):
        """Run a sequence of test commands"""
        
        if self.test_step == 0:
            self.get_logger().info('Test Step 1: Hover at 5m altitude')
            self.send_hover_command(0, 0, 5)
            
        elif self.test_step == 1:
            self.get_logger().info('Test Step 2: Move forward 2m')
            self.send_hover_command(2, 0, 5)
            
        elif self.test_step == 2:
            self.get_logger().info('Test Step 3: Move right 2m')
            self.send_hover_command(2, 2, 5)
            
        elif self.test_step == 3:
            self.get_logger().info('Test Step 4: Return to origin')
            self.send_hover_command(0, 0, 5)
            
        elif self.test_step == 4:
            self.get_logger().info('Test Step 5: Land slowly')
            self.send_hover_command(0, 0, 1)
            
        else:
            self.get_logger().info('Test sequence complete')
            return
            
        self.test_step += 1
        
        # Print current position if available
        if self.current_odom:
            pos = self.current_odom.pose.pose.position
            self.get_logger().info(
                f'Current position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}'
            )
    
    def send_hover_command(self, x, y, z):
        """Send position command for hovering"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = float(z)
        
        # Keep level orientation
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    tester = DroneControlTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
