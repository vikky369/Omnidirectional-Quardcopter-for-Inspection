#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from actuator_msgs.msg import Actuators
import numpy as np

class AdvancedThrusterController(Node):
    def __init__(self):
        super().__init__('advanced_thruster_controller')

        # Publisher for actuator array command
        self.actuator_pub = self.create_publisher(Actuators, '/rotors/command/motor_speed', 10)
        
        # Subscribers for individual thruster feedback
        self.feedback_subs = []
        self.current_speeds = [0.0] * 8  # Store current speeds for each motor
        
        for i in range(1, 9):
            sub = self.create_subscription(
                Float64,
                f'/thruster_{i}/cmd',
                lambda msg, motor_id=i-1: self.feedback_callback(msg, motor_id),
                10
            )
            self.feedback_subs.append(sub)

        # Individual motor target speeds (can be modified for different control)
        self.target_speeds = [5000.0] * 8  # Default hover speeds
        
        # Timer: publish commands at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_commands)
        
        self.get_logger().info('Advanced Thruster Controller started')

    def feedback_callback(self, msg, motor_id):
        """Store feedback from individual motors"""
        self.current_speeds[motor_id] = msg.data

    def set_individual_speeds(self, speeds):
        """Set individual motor speeds - speeds should be list of 8 values"""
        if len(speeds) == 8:
            self.target_speeds = speeds
            self.get_logger().info(f'Updated target speeds: {speeds}')
        else:
            self.get_logger().error('Speed array must have 8 elements')

    def hover_mode(self):
        """Set all motors to hover speed"""
        self.set_individual_speeds([5000.0] * 8)

    def differential_thrust(self, front_speed=5000.0, rear_speed=3000.0):
        """Example: Different speeds for front/rear motors"""
        speeds = [
            front_speed,  # Motor 1 (front-right)
            front_speed,  # Motor 2 (front-left) 
            rear_speed,   # Motor 3 (rear-right)
            rear_speed,   # Motor 4 (rear-left)
            rear_speed,   # Motor 5 (rear-left-bottom)
            front_speed,  # Motor 6 (front-left-bottom)
            front_speed,  # Motor 7 (front-right-bottom)
            rear_speed    # Motor 8 (rear-right-bottom)
        ]
        self.set_individual_speeds(speeds)

    def stop_all_motors(self):
        """Stop all motors"""
        self.set_individual_speeds([0.0] * 8)

    def publish_commands(self):
        """Pack individual speeds into Actuators message and publish"""
        msg = Actuators()
        msg.velocity = self.target_speeds
        
        self.actuator_pub.publish(msg)
        
        # Log current status every 2 seconds
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
        else:
            self.log_counter = 0
            
        if self.log_counter % 40 == 0:  # Every 2 seconds at 20Hz
            self.get_logger().info(f'Target: {[f"{s:.0f}" for s in self.target_speeds]}')
            self.get_logger().info(f'Actual: {[f"{s:.0f}" for s in self.current_speeds]}')

def main(args=None):
    rclpy.init(args=args)
    controller = AdvancedThrusterController()
    
    # Example usage - you can modify this
    try:
        # Start with hover mode
        controller.hover_mode()
        
        # After 5 seconds, try differential thrust
        def switch_to_differential():
            controller.get_logger().info('Switching to differential thrust mode')
            controller.differential_thrust(6000.0, 4000.0)
        
        # Schedule mode change (optional)
        # controller.create_timer(5.0, switch_to_differential)
        
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Stopping all motors...')
        controller.stop_all_motors()
        
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
