#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header
import json
from pathlib import Path

IR_MAX_RAW = 4096.0   


class SensorPublisher(Node):

    def __init__(self):
        super().__init__('sensor_publisher')

        self.sensor_pubs = []
        for i in range(8):
            pub = self.create_publisher(Range, f'/sensor_{i}', 10)
            self.sensor_pubs.append(pub)

        self.timer = self.create_timer(0.1, self.publish_sensors)

        self.shared_file = Path('/shared/sensor_data.json')
        self.loop_count = 0

        self.get_logger().info('Sensor Publisher avviato!')
        self.get_logger().info(f'Lettura da: {self.shared_file}')
        self.get_logger().info('Scala valori: IR raw (0-4096) → range (0.0-1.0)')
        self.get_logger().info('  range ALTO (>0.3) = ostacolo vicino')
        self.get_logger().info('  range BASSO (<0.1) = via libera')

    def publish_sensors(self):
        self.loop_count += 1

        if not self.shared_file.exists():
            if self.loop_count % 100 == 0:
                self.get_logger().warn(f'File non trovato: {self.shared_file}')
            return

        try:
            with open(self.shared_file, 'r') as f:
                data = json.load(f)

            distances = data.get('distances', [])
            if not distances:
                return

            for i, raw_value in enumerate(distances):
                if i >= len(self.sensor_pubs):
                    break


                normalized = float(raw_value) / IR_MAX_RAW
                normalized = max(0.0, min(1.0, normalized))

                msg = Range()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f'sensor_{i}'
                msg.radiation_type = Range.INFRARED
                msg.field_of_view = 0.1
                msg.min_range = 0.0
                msg.max_range = 1.0  
                msg.range = normalized

                self.sensor_pubs[i].publish(msg)

            if self.loop_count % 50 == 0:
                normalized_vals = [f'{v/IR_MAX_RAW:.2f}' for v in distances[:8]]
                self.get_logger().info(f'IR norm: {normalized_vals}')

        except Exception as e:
            if self.loop_count % 100 == 0:
                self.get_logger().error(f'Errore lettura JSON: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Arresto...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
