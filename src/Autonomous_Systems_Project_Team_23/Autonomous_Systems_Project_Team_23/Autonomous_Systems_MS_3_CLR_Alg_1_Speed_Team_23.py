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

"""Closed-loop speed controller node for Team 23 (Milestone 3)."""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def clamp(value, min_value, max_value):
    """Clamp *value* between *min_value* and *max_value*."""
    return max(min_value, min(max_value, value))


def move_towards(current_value, target_value, max_delta):
    """Move *current_value* toward *target_value* by at most *max_delta*."""
    if target_value > current_value:
        return min(current_value + max_delta, target_value)
    return max(current_value - max_delta, target_value)


class AutonomousSystemsMS3CLRAlg1SpeedTeam23(Node):
    """Compute a closed-loop speed command from odometry feedback."""

    def __init__(self):
        super().__init__('autonomous_systems_ms_3_clr_alg_1_speed_team_23')

        self.declare_parameter('state_topic', '/model/vehicle/odometry')
        self.declare_parameter('speed_command_topic', '/ms3/speed_command')
        self.declare_parameter('desired_speed_topic', '/ms4/desired_speed')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('desired_speed', 0.25)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_acceleration', 0.75)
        self.declare_parameter('speed_kp', 1.2)
        self.declare_parameter('speed_ki', 0.35)
        self.declare_parameter('speed_kd', 0.02)
        self.declare_parameter('speed_integral_limit', 1.5)

        self.state_topic = self.get_parameter('state_topic').value
        self.speed_command_topic = self.get_parameter(
            'speed_command_topic'
        ).value
        self.desired_speed_topic = self.get_parameter(
            'desired_speed_topic'
        ).value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.desired_speed = float(self.get_parameter('desired_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_acceleration = float(
            self.get_parameter('max_acceleration').value
        )
        self.speed_kp = float(self.get_parameter('speed_kp').value)
        self.speed_ki = float(self.get_parameter('speed_ki').value)
        self.speed_kd = float(self.get_parameter('speed_kd').value)
        self.speed_integral_limit = float(
            self.get_parameter('speed_integral_limit').value
        )

        self.speed_command_publisher = self.create_publisher(
            Float64,
            self.speed_command_topic,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.state_topic,
            self.odom_callback,
            10,
        )
        self.desired_speed_subscription = self.create_subscription(
            Float64,
            self.desired_speed_topic,
            self.desired_speed_callback,
            10,
        )

        self.actual_speed = 0.0
        self.error_integral = 0.0
        self.previous_error = 0.0
        self.last_speed_command = 0.0
        self.last_control_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.control_timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            'MS3 speed controller ready. state_topic=%s, speed_command_topic=%s, '
            'desired_speed_topic=%s, default_desired_speed=%.2f m/s.'
            % (
                self.state_topic,
                self.speed_command_topic,
                self.desired_speed_topic,
                self.desired_speed,
            )
        )

    def odom_callback(self, msg):
        """Update actual speed using odometry feedback."""
        self.actual_speed = float(msg.twist.twist.linear.x)

    def desired_speed_callback(self, msg):
        """Update the desired speed setpoint from the planning node."""
        self.desired_speed = clamp(float(msg.data), -self.max_speed, self.max_speed)

    def control_loop(self):
        """Run PI/PID speed control and publish commanded speed."""
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1_000_000_000.0
        if dt <= 0.0:
            return

        error = self.desired_speed - self.actual_speed
        self.error_integral = clamp(
            self.error_integral + error * dt,
            -self.speed_integral_limit,
            self.speed_integral_limit,
        )
        error_derivative = (error - self.previous_error) / dt

        correction = (
            self.speed_kp * error
            + self.speed_ki * self.error_integral
            + self.speed_kd * error_derivative
        )

        speed_command = self.desired_speed + correction
        speed_command = clamp(speed_command, -self.max_speed, self.max_speed)

        max_delta = self.max_acceleration * dt
        speed_command = move_towards(
            self.last_speed_command,
            speed_command,
            max_delta,
        )

        speed_message = Float64()
        speed_message.data = speed_command
        self.speed_command_publisher.publish(speed_message)

        self.last_speed_command = speed_command
        self.previous_error = error
        self.last_control_time = now

        elapsed = (now - self.last_log_time).nanoseconds / 1_000_000_000.0
        if elapsed >= 0.5:
            self.get_logger().info(
                'speed | desired=%.2f m/s, actual=%.2f m/s, error=%.2f, '
                'command=%.2f m/s'
                % (
                    self.desired_speed,
                    self.actual_speed,
                    error,
                    speed_command,
                )
            )
            self.last_log_time = now

    def stop_controller(self):
        """Publish a zero speed command when shutting down."""
        if not rclpy.ok():
            return

        stop_message = Float64()
        stop_message.data = 0.0
        self.speed_command_publisher.publish(stop_message)


def main(args=None):
    """Entry point for the MS3 speed controller node."""
    rclpy.init(args=args)
    node = AutonomousSystemsMS3CLRAlg1SpeedTeam23()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_controller()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
