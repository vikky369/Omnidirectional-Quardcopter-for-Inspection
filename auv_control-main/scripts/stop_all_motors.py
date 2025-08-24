#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MotorStopper(Node):
    def __init__(self):
        super().__init__('motor_stopper')
        
        # Create publishers for all 8 motor joints
        self.motor_publishers = []
        for i in range(1, 9):
            topic = f'/model/drone_frame/joint/thruster_joint_{i}/cmd_vel'
            pub = self.create_publisher(Float64, topic, 10)
            self.motor_publishers.append(pub)
        
        # Send zero velocities
        self.stop_motors()
        
    def stop_motors(self):
        zero_msg = Float64()
        zero_msg.data = 0.0
        
        for pub in self.motor_publishers:
            pub.publish(zero_msg)
        
        self.get_logger().info('Sent zero velocity commands to all motors')

def main(args=None):
    rclpy.init(args=args)
    motor_stopper = MotorStopper()
    
    # Keep sending zeros for a few seconds
    for _ in range(10):
        motor_stopper.stop_motors()
        rclpy.spin_once(motor_stopper, timeout_sec=0.1)
    
    motor_stopper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
