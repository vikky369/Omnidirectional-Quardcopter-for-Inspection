#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np

class ThrusterCommandMapper(Node):
    def __init__(self):
        super().__init__('thruster_command_mapper')
        
        # Subscribe to thrust commands from thruster manager
        self.thrust_sub = self.create_subscription(
            JointState,
            'thrust_commands',
            self.thrust_callback,
            10
        )
        
        # Publishers for individual motor velocity commands
        self.motor_pubs = {}
        for i in range(1, 9):
            topic = f'/model/drone_frame/joint/thruster_joint_{i}/cmd_vel'
            self.motor_pubs[f'thruster_joint_{i}'] = self.create_publisher(
                Float64, topic, 10
            )
        
        # Thrust to velocity conversion parameters
        self.thrust_to_vel_ratio = 100.0  # Adjust based on motor characteristics
        self.max_velocity = 8000.0        # Maximum motor velocity (rad/s)
        
        self.get_logger().info('Thruster Command Mapper initialized')
        
    def thrust_callback(self, msg):
        """Convert thrust commands to motor velocity commands"""
        
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.motor_pubs:
                # Get thrust command (from effort field)
                thrust = msg.effort[i] if i < len(msg.effort) else 0.0
                
                # Convert thrust to motor velocity
                # Positive thrust = positive rotation
                velocity = thrust * self.thrust_to_vel_ratio
                
                # Apply velocity limits
                velocity = np.clip(velocity, -self.max_velocity, self.max_velocity)
                
                # Publish velocity command
                vel_msg = Float64()
                vel_msg.data = float(velocity)
                self.motor_pubs[joint_name].publish(vel_msg)
        
        # Log thrust commands for debugging
        if len(msg.effort) > 0:
            thrust_sum = sum(msg.effort)
            self.get_logger().info(
                f'Total thrust: {thrust_sum:.2f}N, Motor commands sent',
                throttle_duration_sec=1.0
            )

def main(args=None):
    rclpy.init(args=args)
    mapper = ThrusterCommandMapper()
    
    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        pass
    finally:
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
