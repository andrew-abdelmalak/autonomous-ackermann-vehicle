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

"""Publisher-subscriber demo node for Team 23 (Milestone 1)."""

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class VehiclePubSubNode(Node):
    """Publishes velocity commands and subscribes to odometry at 1 Hz."""

    def __init__(self):
        super().__init__('vehicle_pub_sub_node')
        self.declare_parameter('command_topic', '/model/vehicle/cmd_vel')
        self.declare_parameter('state_topic', '/model/vehicle/odometry')
        cmd_topic = self.get_parameter('command_topic').value
        state_topic = self.get_parameter('state_topic').value

        self.publisher_ = self.create_publisher(Twist, cmd_topic, 10)
        self.subscription = self.create_subscription(
            Odometry, state_topic, self.odom_callback, 10
        )
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """Publish a constant velocity command."""
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.2
        self.publisher_.publish(msg)
        self.get_logger().info(
            'Publishing: linear=%.2f, angular=%.2f'
            % (msg.linear.x, msg.angular.z)
        )

    def odom_callback(self, msg):
        """Log the received odometry position."""
        pos = msg.pose.pose.position
        self.get_logger().info(
            'Odometry: position=(%.2f, %.2f, %.2f)'
            % (pos.x, pos.y, pos.z)
        )


def main(args=None):
    """Entry point for the publisher-subscriber node."""
    rclpy.init(args=args)
    node = VehiclePubSubNode()
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
