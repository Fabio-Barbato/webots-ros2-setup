#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import time


OBSTACLE_THRESHOLD = 0.15   
                             
                             
LINEAR_SPEED       = 0.15
ANGULAR_SPEED      = 1.2
MIN_TURN_TIME      = 1.5

FRONT_SENSORS = [0, 1, 7]
LEFT_SENSORS  = [5, 6, 7]
RIGHT_SENSORS = [0, 1, 2]


class ObstacleAvoider(Node):

    def __init__(self):
        super().__init__('obstacle_avoider')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sensor_values = [0.0] * 8
        self.sensors_received = False

        for i in range(8):
            self.create_subscription(
                Range,
                f'/sensor_{i}',
                lambda msg, idx=i: self._sensor_cb(msg, idx),
                qos
            )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.turning         = False
        self.turn_start_time = None
        self.turn_direction  = 1

        self.timer = self.create_timer(0.1, self.control_loop)


    def _sensor_cb(self, msg: Range, idx: int):
        self.sensor_values[idx] = msg.range
        self.sensors_received = True

    def _obstacle_ahead(self) -> bool:
        return any(self.sensor_values[i] > OBSTACLE_THRESHOLD for i in FRONT_SENSORS)

    def _best_turn_direction(self) -> int:
        left_score  = sum(self.sensor_values[i] for i in LEFT_SENSORS)
        right_score = sum(self.sensor_values[i] for i in RIGHT_SENSORS)
        return 1 if left_score <= right_score else -1

    def control_loop(self):
        if not self.sensors_received:
            return

        cmd = Twist()
        obstacle = self._obstacle_ahead()

        if self.turning:
            elapsed = time.time() - self.turn_start_time

            if elapsed < MIN_TURN_TIME or obstacle:
                cmd.linear.x  = 0.0
                cmd.angular.z = ANGULAR_SPEED * self.turn_direction
                self.get_logger().info(
                    f'Turn {"←" if self.turn_direction > 0 else "→"} '
                    f'{elapsed:.1f}s/{MIN_TURN_TIME}s | '
                    f'IR front: {[f"{self.sensor_values[i]:.3f}" for i in FRONT_SENSORS]}',
                    throttle_duration_sec=0.4
                )
            else:
                self.turning = False
                self.get_logger().info('Clear')

        elif obstacle:
            self.turn_direction  = self._best_turn_direction()
            self.turning         = True
            self.turn_start_time = time.time()
            cmd.linear.x  = 0.0
            cmd.angular.z = ANGULAR_SPEED * self.turn_direction
            self.get_logger().info(
                f'OBSTACLE: {[f"{self.sensor_values[i]:.3f}" for i in FRONT_SENSORS]} '
                f'→ Turn to {"left" if self.turn_direction > 0 else "right"}'
            )

        else:
            cmd.linear.x  = LINEAR_SPEED
            cmd.angular.z = 0.0
            self.get_logger().info(
                f'→ Walking: {[f"{self.sensor_values[i]:.3f}" for i in FRONT_SENSORS]}',
                throttle_duration_sec=1.0
            )

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stop...')
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
