#!/usr/bin/env python3
"""
Sensor Publisher Node

Reads sensor_data.json and publishes ROS2 Range messages.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header
import json
from pathlib import Path


class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        
        # Create 8 publishers (one per sensor)
        self.sensor_pubs = []
        for i in range(8):
            pub = self.create_publisher(Range, f'/sensor_{i}', 10)
            self.sensor_pubs.append(pub)
        
        # Timer: publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_sensors)
        
        # Shared file path
        self.shared_file = Path('/shared/sensor_data.json')
        
        self.get_logger().info('Sensor Publisher started!')
        self.get_logger().info(f'Reading from: {self.shared_file}')
        
        self.loop_count = 0
    
    def publish_sensors(self):
        self.loop_count += 1
        
        # Check file exists
        if not self.shared_file.exists():
            if self.loop_count % 100 == 0:
                self.get_logger().warn(f'File not found: {self.shared_file}')
            return
        
        try:
            # Read JSON
            with open(self.shared_file, 'r') as f:
                data = json.load(f)
            
            distances = data.get('distances', [])
            
            if len(distances) == 0:
                return
            
            # Publish each sensor
            for i, distance in enumerate(distances):
                if i >= len(self.sensor_pubs):
                    break
                
                msg = Range()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f'sensor_{i}'
                msg.radiation_type = Range.INFRARED
                msg.field_of_view = 0.1
                msg.min_range = 0.0
                msg.max_range = 2.0
                msg.range = float(distance) / 1000.0  # mm to m
                
                self.sensor_pubs[i].publish(msg)
            
            # Log occasionally
            if self.loop_count % 50 == 0:
                min_d = data.get('min_distance', 0) / 1000.0
                max_d = data.get('max_distance', 0) / 1000.0
                self.get_logger().info(
                    f'📊 Publishing: min={min_d:.3f}m, max={max_d:.3f}m'
                )
                    
        except Exception as e:
            if self.loop_count % 100 == 0:
                self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
