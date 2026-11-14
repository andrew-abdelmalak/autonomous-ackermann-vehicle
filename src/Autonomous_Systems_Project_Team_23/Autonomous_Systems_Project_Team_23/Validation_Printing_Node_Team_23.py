# Copyright 2026 Team 23
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Validation printing node for Team 23 (Milestone 1)."""

import rclpy
from rclpy.node import Node


class ValidationPrintingNode(Node):
    """Periodically logs a validation message at 1 Hz."""

    def __init__(self):
        super().__init__('validation_printing_node')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('ROS 2 Jazzy is running successfully on Team 23')

    def timer_callback(self):
        """Log a periodic validation message."""
        self.get_logger().info('ROS 2 Jazzy is running successfully on Team 23')


def main(args=None):
    """Entry point for the validation printing node."""
    rclpy.init(args=args)
    node = ValidationPrintingNode()
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
