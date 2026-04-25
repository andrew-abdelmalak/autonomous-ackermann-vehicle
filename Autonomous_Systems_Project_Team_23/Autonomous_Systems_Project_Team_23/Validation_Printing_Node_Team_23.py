# Copyright 2026 Team 23
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Validation printing node for Team 23 (Milestone 1)."""

import rclpy
from rclpy.node import Node


class ValidationPrintingNode(Node):
    """Periodically logs a validation message at 1 Hz."""

    def __init__(self):
        super().__init__('validation_printing_node')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(
            'ROS 2 Jazzy is running successfully on Team 23'
        )

    def timer_callback(self):
        """Log a periodic validation message."""
        self.get_logger().info(
            'ROS 2 Jazzy is running successfully on Team 23'
        )


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
