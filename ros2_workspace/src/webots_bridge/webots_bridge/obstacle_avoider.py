#!/usr/bin/env python3
"""
Obstacle Avoider Example

Simple algorithm that avoids obstacles using front sensor.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # QoS for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to front sensor
        self.sensor_sub = self.create_subscription(
            Range,
            '/sensor_0',
            self.sensor_callback,
            qos_profile
        )
        
        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Obstacle threshold (E-puck IR sensors: high value = obstacle)
        self.obstacle_threshold = 0.2  # meters
        
        self.get_logger().info('🚧 Obstacle Avoider Started!')
        self.get_logger().info(f'   Threshold: > {self.obstacle_threshold}m = obstacle')
        
        self.received_count = 0
    
    def sensor_callback(self, msg):
        self.received_count += 1
        
        cmd = Twist()
        
        # E-puck IR: HIGH values = obstacle close
        if msg.range > self.obstacle_threshold:
            # Obstacle detected - turn!
            self.get_logger().info(
                f'⚠️  OBSTACLE {msg.range:.3f}m - TURNING!'
            )
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn left
        else:
            # Clear - go forward!
            if self.received_count % 20 == 0:
                self.get_logger().info(
                    f'✅ FORWARD {msg.range:.3f}m'
                )
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
        
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
