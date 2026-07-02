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

"""Stanley-style lateral controller node for Team 23."""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


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


def normalize_angle(angle_rad):
    """Normalize angle to [-pi, pi]."""
    wrapped = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def path_normal_coordinate(x_position, y_position, path_heading):
    """Project the vehicle position onto the path normal."""
    return (
        -math.sin(path_heading) * x_position
        + math.cos(path_heading) * y_position
    )


def move_towards(current_value, target_value, max_delta):
    """Move *current_value* toward *target_value* by at most *max_delta*."""
    if target_value > current_value:
        return min(current_value + max_delta, target_value)
    return max(current_value - max_delta, target_value)


def steering_to_yaw_rate(speed, steering_angle, wheel_base, max_turn_rate):
    """Convert steering-angle intent to a simulator yaw-rate command."""
    if abs(wheel_base) < 1e-6:
        return 0.0

    yaw_rate = speed * math.tan(steering_angle) / wheel_base
    return clamp(yaw_rate, -max_turn_rate, max_turn_rate)


class AutonomousSystemsMS3CLRAlg2LateralTeam23(Node):
    """Compute lateral correction and publish the final command."""

    def __init__(self):
        super().__init__('autonomous_systems_ms_3_clr_alg_2_lateral_team_23')

        self.declare_parameter('command_topic', '/model/vehicle/cmd_vel')
        self.declare_parameter('state_topic', '/model/vehicle/odometry')
        self.declare_parameter('speed_command_topic', '/ms3/speed_command')
        self.declare_parameter('desired_lane_topic', '/ms4/desired_lane')
        self.declare_parameter('desired_heading_topic', '/ms4/desired_heading')
        self.declare_parameter('steering_command_topic', '/ms3/steering_command')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('desired_lane', 0.0)
        self.declare_parameter('desired_heading', 0.0)
        self.declare_parameter('wheel_base', 1.0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_turn_rate', 0.5)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('max_steering_rate', 1.0)
        self.declare_parameter('lateral_kp', 0.95)
        self.declare_parameter('lateral_ki', 0.02)
        self.declare_parameter('lateral_kd', 0.18)
        self.declare_parameter('lateral_integral_limit', 2.0)
        self.declare_parameter('heading_kp', 1.2)
        self.declare_parameter('heading_kd', 0.25)

        self.command_topic = self.get_parameter('command_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.speed_command_topic = self.get_parameter(
            'speed_command_topic'
        ).value
        self.desired_lane_topic = self.get_parameter(
            'desired_lane_topic'
        ).value
        self.desired_heading_topic = self.get_parameter(
            'desired_heading_topic'
        ).value
        self.steering_command_topic = self.get_parameter(
            'steering_command_topic'
        ).value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.desired_lane = float(self.get_parameter('desired_lane').value)
        self.desired_heading = float(
            self.get_parameter('desired_heading').value
        )
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_turn_rate = float(self.get_parameter('max_turn_rate').value)
        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value
        )
        self.max_steering_rate = float(
            self.get_parameter('max_steering_rate').value
        )
        self.lateral_kp = float(self.get_parameter('lateral_kp').value)
        self.lateral_ki = float(self.get_parameter('lateral_ki').value)
        self.lateral_kd = float(self.get_parameter('lateral_kd').value)
        self.lateral_integral_limit = float(
            self.get_parameter('lateral_integral_limit').value
        )
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.heading_kd = float(self.get_parameter('heading_kd').value)

        self.command_publisher = self.create_publisher(
            Twist,
            self.command_topic,
            10,
        )
        self.steering_command_publisher = self.create_publisher(
            Float64,
            self.steering_command_topic,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.state_topic,
            self.odom_callback,
            10,
        )
        self.speed_subscription = self.create_subscription(
            Float64,
            self.speed_command_topic,
            self.speed_command_callback,
            10,
        )
        self.desired_lane_subscription = self.create_subscription(
            Float64,
            self.desired_lane_topic,
            self.desired_lane_callback,
            10,
        )
        self.desired_heading_subscription = self.create_subscription(
            Float64,
            self.desired_heading_topic,
            self.desired_heading_callback,
            10,
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_yaw_rate = 0.0
        self.current_speed = 0.0
        self.current_speed_command = 0.0
        self.lateral_error_integral = 0.0
        self.applied_steering = 0.0
        self.last_control_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.control_timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            'MS3 lateral controller ready. command_topic=%s, desired_lane_topic=%s, '
            'desired_heading_topic=%s.'
            % (
                self.command_topic,
                self.desired_lane_topic,
                self.desired_heading_topic,
            )
        )

    def odom_callback(self, msg):
        """Update current lateral position from odometry."""
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.current_yaw_rate = float(msg.twist.twist.angular.z)
        self.current_speed = float(msg.twist.twist.linear.x)

    def speed_command_callback(self, msg):
        """Receive latest closed-loop speed command from the speed node."""
        self.current_speed_command = float(msg.data)

    def desired_lane_callback(self, msg):
        """Receive the planned path offset from the planning node."""
        self.desired_lane = float(msg.data)

    def desired_heading_callback(self, msg):
        """Receive the planned path heading from the planning node."""
        self.desired_heading = normalize_angle(float(msg.data))

    def control_loop(self):
        """Run lateral control and publish command to the vehicle."""
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1_000_000_000.0
        if dt <= 0.0:
            return

        path_coordinate = path_normal_coordinate(
            self.current_x,
            self.current_y,
            self.desired_heading,
        )
        lateral_error = self.desired_lane - path_coordinate
        self.lateral_error_integral = clamp(
            self.lateral_error_integral + lateral_error * dt,
            -self.lateral_integral_limit,
            self.lateral_integral_limit,
        )
        heading_error = normalize_angle(self.desired_heading - self.current_yaw)
        speed_softening = max(abs(self.current_speed), max(self.lateral_kd, 0.05))
        stanley_term = math.atan2(
            self.lateral_kp * lateral_error,
            speed_softening,
        )

        raw_steering = (
            self.heading_kp * heading_error
            + stanley_term
            + self.lateral_ki * self.lateral_error_integral
            - self.heading_kd * self.current_yaw_rate
        )
        raw_steering = clamp(
            raw_steering,
            -self.max_steering_angle,
            self.max_steering_angle,
        )

        max_steering_delta = self.max_steering_rate * dt
        self.applied_steering = move_towards(
            self.applied_steering,
            raw_steering,
            max_steering_delta,
        )

        speed_command = clamp(
            self.current_speed_command,
            -self.max_speed,
            self.max_speed,
        )

        steering_message = Float64()
        steering_message.data = self.applied_steering
        self.steering_command_publisher.publish(steering_message)

        yaw_rate = steering_to_yaw_rate(
            speed_command,
            self.applied_steering,
            self.wheel_base,
            self.max_turn_rate,
        )

        command = Twist()
        command.linear.x = speed_command
        command.angular.z = yaw_rate
        self.command_publisher.publish(command)

        self.last_control_time = now

        elapsed = (now - self.last_log_time).nanoseconds / 1_000_000_000.0
        if elapsed >= 0.5:
            self.get_logger().info(
                'lateral | desired_lane=%.2f m, path_coordinate=%.2f m, '
                'error=%.2f m, desired_heading=%.2f deg, yaw=%.2f deg, '
                'steering=%.2f rad, speed_cmd=%.2f m/s'
                % (
                    self.desired_lane,
                    path_coordinate,
                    lateral_error,
                    math.degrees(self.desired_heading),
                    math.degrees(self.current_yaw),
                    self.applied_steering,
                    speed_command,
                )
            )
            self.last_log_time = now

    def stop_vehicle(self):
        """Publish a zero command when shutting down."""
        if rclpy.ok():
            self.command_publisher.publish(Twist())


def main(args=None):
    """Entry point for the MS3 lateral controller node."""
    rclpy.init(args=args)
    node = AutonomousSystemsMS3CLRAlg2LateralTeam23()
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
