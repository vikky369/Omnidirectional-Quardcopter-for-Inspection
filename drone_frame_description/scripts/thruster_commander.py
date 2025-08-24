#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ThrusterCommander(Node):
    def __init__(self):
        super().__init__('thruster_commander')

        # Publishers for all 8 thrusters
        self.thruster_pubs = [
            self.create_publisher(Float64, f'/thruster_{i}/command', 10)
            for i in range(1, 9)
        ]

        # Fixed thrust value for 10kg drone hover
        self.thrust_value = 5000.0

        # Timer: publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_thrust)

    def publish_thrust(self):
        msg = Float64()
        msg.data = self.thrust_value
        for i, pub in enumerate(self.thruster_pubs, start=1):
            pub.publish(msg)
        self.get_logger().info(f'Published {msg.data} to all thrusters')

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
