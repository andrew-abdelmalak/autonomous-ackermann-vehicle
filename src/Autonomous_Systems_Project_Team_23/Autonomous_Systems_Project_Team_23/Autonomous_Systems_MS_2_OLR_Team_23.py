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

"""Open-loop response (OLR) driving node for Team 23 (Milestone 2)."""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def clamp(value, min_value, max_value):
    """Clamp *value* between *min_value* and *max_value*."""
    return max(min_value, min(max_value, value))


def quaternion_to_yaw(quaternion):
    """Convert a quaternion to a yaw angle in radians."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def steering_to_yaw_rate(speed, steering_angle, wheel_base, max_turn_rate):
    """Convert steering-angle intent to a simulator yaw-rate command."""
    if abs(wheel_base) < 1e-6:
        return 0.0

    yaw_rate = speed * math.tan(steering_angle) / wheel_base
    return clamp(yaw_rate, -max_turn_rate, max_turn_rate)


class AutonomousSystemsMS2OLRTeam23(Node):
    """Publish constant speed and steering commands (open-loop)."""

    def __init__(self):
        super().__init__('autonomous_systems_ms_2_olr_team_23')

        self.declare_parameter('command_topic', '/model/vehicle/cmd_vel')
        self.declare_parameter('state_topic', '/model/vehicle/odometry')
        self.declare_parameter('joint_state_topic', 'joint_states')
        self.declare_parameter(
            'left_steering_joint_name',
            'front_left_wheel_steering_joint',
        )
        self.declare_parameter(
            'right_steering_joint_name',
            'front_right_wheel_steering_joint',
        )
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('wheel_base', 1.0)
        self.declare_parameter('desired_speed', 0.25)
        self.declare_parameter('desired_steering', 0.0)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_turn_rate', 0.5)

        self.command_topic = self.get_parameter('command_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.left_steering_joint_name = self.get_parameter(
            'left_steering_joint_name'
        ).value
        self.right_steering_joint_name = self.get_parameter(
            'right_steering_joint_name'
        ).value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.desired_speed = float(self.get_parameter('desired_speed').value)
        self.desired_steering = float(
            self.get_parameter('desired_steering').value
        )
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_turn_rate = float(self.get_parameter('max_turn_rate').value)

        self.command_publisher = self.create_publisher(
            Twist, self.command_topic, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.state_topic,
            self.odom_callback,
            10,
        )
        self.joint_state_subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.current_command = Twist()
        self.current_steering_angle = 0.0
        self.last_state_log_time = self.get_clock().now()

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.publish_timer = self.create_timer(period, self.publish_command)

        self.get_logger().info(
            'MS2 OLR ready. Publishing to %s and subscribing to %s '
            '(desired_speed=%.2f, desired_steering=%.2f).'
            % (
                self.command_topic,
                self.state_topic,
                self.desired_speed,
                self.desired_steering,
            )
        )

    def publish_command(self):
        """Publish the constant open-loop command."""
        command = Twist()
        command.linear.x = clamp(
            self.desired_speed, -self.max_speed, self.max_speed
        )
        command.angular.z = steering_to_yaw_rate(
            command.linear.x,
            self.desired_steering,
            self.wheel_base,
            self.max_turn_rate,
        )
        self.current_command = command
        self.command_publisher.publish(command)

    def joint_state_callback(self, msg):
        """Update the averaged steering angle from joint states."""
        joint_positions = dict(zip(msg.name, msg.position))
        steering_values = []

        if self.left_steering_joint_name in joint_positions:
            steering_values.append(
                joint_positions[self.left_steering_joint_name]
            )
        if self.right_steering_joint_name in joint_positions:
            steering_values.append(
                joint_positions[self.right_steering_joint_name]
            )

        if steering_values:
            self.current_steering_angle = (
                sum(steering_values) / len(steering_values)
            )

    def odom_callback(self, msg):
        """Log vehicle state at a throttled rate."""
        now = self.get_clock().now()
        elapsed_since_log = (
            now - self.last_state_log_time
        ).nanoseconds / 1_000_000_000.0
        if elapsed_since_log < 0.5:
            return

        position = msg.pose.pose.position
        velocity = msg.twist.twist.linear.x
        yaw_deg = math.degrees(
            quaternion_to_yaw(msg.pose.pose.orientation)
        )

        self.get_logger().info(
            'state | steering=%.2f rad, velocity=%.2f m/s, '
            'position=(%.2f, %.2f, %.2f), orientation_yaw=%.2f deg'
            % (
                self.current_steering_angle,
                velocity,
                position.x,
                position.y,
                position.z,
                yaw_deg,
            )
        )
        self.last_state_log_time = now

    def stop_vehicle(self):
        """Publish a zero-velocity command to stop the vehicle."""
        if rclpy.ok():
            self.command_publisher.publish(Twist())
            self.get_logger().info('Vehicle stop command sent.')


def main(args=None):
    """Entry point for the OLR driving node."""
    rclpy.init(args=args)
    node = AutonomousSystemsMS2OLRTeam23()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_vehicle()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
