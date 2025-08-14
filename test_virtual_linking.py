#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math
import time

class VirtualLinkingTester(Node):
    def __init__(self):
        super().__init__('virtual_linking_tester')
        
        # Publishers for joint commands
        self.joint_publishers = {}
        
        # Main actuation joints
        main_joints = ['revolute_1', 'revolute_2', 'revolute_3', 'revolute_4', 
                      'revolute_5', 'revolute_6', 'revolute_7', 'revolute_8']
        
        for joint in main_joints:
            self.joint_publishers[joint] = self.create_publisher(
                Float64, f'/drone_frame/{joint}_position_controller/commands', 10
            )
        
        # Virtual linking joints
        virtual_joints = ['revolute_9', 'revolute_10', 'revolute_11', 'revolute_12',
                         'revolute_13', 'revolute_14', 'revolute_15', 'revolute_16']
        
        for joint in virtual_joints:
            self.joint_publishers[joint] = self.create_publisher(
                Float64, f'/drone_frame/{joint}_position_controller/commands', 10
            )
        
        # Timer for periodic testing
        self.timer = self.create_timer(5.0, self.test_virtual_linking)
        self.test_step = 0
        
        self.get_logger().info('Virtual Linking Tester started')
        self.get_logger().info('Testing sequence will begin in 5 seconds...')
    
    def test_virtual_linking(self):
        """Test different virtual linking scenarios"""
        self.test_step += 1
        
        if self.test_step == 1:
            self.get_logger().info('Test 1: Moving arm_5 (revolute_4) - should move holder_1')
            self.move_joint('revolute_4', 0.5)
            
        elif self.test_step == 2:
            self.get_logger().info('Test 2: Moving arm_6 (revolute_3) - should also move holder_1')
            self.move_joint('revolute_3', 0.5)
            
        elif self.test_step == 3:
            self.get_logger().info('Test 3: Moving arm_1 (revolute_8) - should move holder_3')
            self.move_joint('revolute_8', 0.5)
            
        elif self.test_step == 4:
            self.get_logger().info('Test 4: Moving arm_2 (revolute_7) - should also move holder_3')
            self.move_joint('revolute_7', 0.5)
            
        elif self.test_step == 5:
            self.get_logger().info('Test 5: Moving arm_3 (revolute_5) - should move holder_2')
            self.move_joint('revolute_5', 0.5)
            
        elif self.test_step == 6:
            self.get_logger().info('Test 6: Moving arm_4 (revolute_6) - should also move holder_2')
            self.move_joint('revolute_6', 0.5)
            
        elif self.test_step == 7:
            self.get_logger().info('Test 7: Moving arm_7 (revolute_2) - should move holder_4')
            self.move_joint('revolute_2', 0.5)
            
        elif self.test_step == 8:
            self.get_logger().info('Test 8: Moving arm_8 (revolute_1) - should also move holder_4')
            self.move_joint('revolute_1', 0.5)
            
        elif self.test_step == 9:
            self.get_logger().info('Test 9: Resetting all joints to zero')
            self.reset_all_joints()
            self.test_step = 0  # Reset for continuous testing
    
    def move_joint(self, joint_name, position):
        """Move a specific joint to a position"""
        if joint_name in self.joint_publishers:
            msg = Float64()
            msg.data = position
            self.joint_publishers[joint_name].publish(msg)
            self.get_logger().info(f'Moving {joint_name} to position {position}')
        else:
            self.get_logger().warn(f'Publisher not found for joint {joint_name}')
    
    def reset_all_joints(self):
        """Reset all joints to zero position"""
        for joint_name in self.joint_publishers:
            self.move_joint(joint_name, 0.0)
        self.get_logger().info('All joints reset to zero')

def main(args=None):
    rclpy.init(args=args)
    
    tester = VirtualLinkingTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

