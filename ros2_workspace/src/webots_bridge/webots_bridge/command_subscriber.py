#!/usr/bin/env python3
"""
Command Subscriber Node

Subscribes to /cmd_vel and writes to cmd_vel.json.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
from pathlib import Path


class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('command_subscriber')
        
        # Subscribe to velocity commands
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        
        # Shared file path
        self.shared_file = Path('/shared/cmd_vel.json')
        
        self.get_logger().info('Command Subscriber started!')
        self.get_logger().info(f'Writing to: {self.shared_file}')
    
    def cmd_callback(self, msg):
        try:
            data = {
                'linear': float(msg.linear.x),
                'angular': float(msg.angular.z)
            }
            
            with open(self.shared_file, 'w') as f:
                json.dump(data, f)
            
            self.get_logger().info(
                f'CMD: linear={data["linear"]:+.2f}, angular={data["angular"]:+.2f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CommandSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
