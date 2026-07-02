#!/usr/bin/env python3
"""Record Twist commands to CSV for replay."""

import csv
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdRecorder(Node):
    def __init__(self):
        super().__init__('ms4_cmd_record_team_23')

        self.declare_parameter('input_topic', '/model/vehicle/cmd_vel')
        self.declare_parameter(
            'output_path',
            os.path.expanduser('~/ms4_cmd_record.csv')
        )

        self._topic = self.get_parameter('input_topic').value
        raw_path = self.get_parameter('output_path').value
        self._output_path = os.path.expanduser(raw_path)
        self._start_time = time.monotonic()

        try:
            output_dir = os.path.dirname(self._output_path)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
            self._file = open(self._output_path, 'w', newline='')
        except Exception as exc:
            self.get_logger().error(
                f'Failed to open {self._output_path}: {exc}'
            )
            raise
        self._writer = csv.writer(self._file)
        self._writer.writerow(['t', 'linear_x', 'angular_z'])

        self.create_subscription(Twist, self._topic, self._on_cmd, 50)

        self.get_logger().info(
            f'Recording {self._topic} to {self._output_path}'
        )

    def _on_cmd(self, msg: Twist):
        t = time.monotonic() - self._start_time
        self._writer.writerow([f'{t:.6f}', f'{msg.linear.x:.6f}', f'{msg.angular.z:.6f}'])
        self._file.flush()

    def destroy_node(self):
        try:
            self._file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
