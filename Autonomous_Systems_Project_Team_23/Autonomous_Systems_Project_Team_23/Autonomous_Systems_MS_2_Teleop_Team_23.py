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

"""Teleoperation driving node for Team 23 (Milestone 2)."""

import math
import select
import sys
import termios
import tty

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


def move_towards(current_value, target_value, max_delta):
    """Move *current_value* toward *target_value* by at most *max_delta*."""
    if target_value > current_value:
        return min(current_value + max_delta, target_value)
    return max(current_value - max_delta, target_value)


class AutonomousSystemsMS2TeleopTeam23(Node):
    """Drive the vehicle interactively via keyboard arrow keys."""

    def __init__(self):
        super().__init__('autonomous_systems_ms_2_teleop_team_23')

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
        self.declare_parameter('initial_speed', 0.0)
        self.declare_parameter('lane', 0.0)
        self.declare_parameter('desired_speed', 0.25)
        self.declare_parameter('desired_lane', 0.0)
        self.declare_parameter('linear_step', 0.05)
        self.declare_parameter('steering_step', 0.10)
        self.declare_parameter('steering_rate_limit', 1.0)
        self.declare_parameter('wheel_base', 1.0)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_turn_rate', 0.5)
        self.declare_parameter('serial_forwarding_enabled', False)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baudrate', 115200)

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
        self.linear_step = float(self.get_parameter('linear_step').value)
        self.steering_step = float(self.get_parameter('steering_step').value)
        self.steering_rate_limit = float(
            self.get_parameter('steering_rate_limit').value
        )
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_turn_rate = float(self.get_parameter('max_turn_rate').value)
        self.current_linear = float(self.get_parameter('initial_speed').value)
        self.current_angular = 0.0
        self.applied_steering = 0.0
        self.lane = float(self.get_parameter('lane').value)
        self.desired_speed = float(self.get_parameter('desired_speed').value)
        self.desired_lane = float(self.get_parameter('desired_lane').value)
        self.serial_forwarding_enabled = bool(
            self.get_parameter('serial_forwarding_enabled').value
        )
        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baudrate = int(self.get_parameter('serial_baudrate').value)

        self.command_publisher = self.create_publisher(Twist, self.command_topic, 10)
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

        self.serial_connection = None
        self.terminal_settings = None
        self.current_steering_angle = 0.0
        self.last_state_log_time = self.get_clock().now()
        self.shutdown_requested = False

        self.setup_terminal()
        self.setup_serial_forwarding()

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.publish_timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            'MS2 Teleop ready. Up/Down changes speed, Left/Right changes steering, '
            'Space stops, Q quits. Publishing to %s and subscribing to %s '
            '(initial_speed=%.2f, lane=%.2f, desired_speed=%.2f, desired_lane=%.2f).'
            % (
                self.command_topic,
                self.state_topic,
                self.current_linear,
                self.lane,
                self.desired_speed,
                self.desired_lane,
            )
        )

    def setup_terminal(self):
        """Switch stdin to cbreak mode for raw key reading."""
        if not sys.stdin.isatty():
            self.get_logger().warning(
                'No interactive terminal detected. '
                'Teleop will keep publishing the '
                'current command only.'
            )
            return

        self.terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def setup_serial_forwarding(self):
        """Open the serial port for Arduino forwarding if enabled."""
        if not self.serial_forwarding_enabled:
            return

        try:
            import serial
        except ImportError:
            self.get_logger().warning(
                'serial_forwarding_enabled=true but '
                'pyserial is not installed. Simulation '
                'will continue without serial forwarding.'
            )
            return

        try:
            self.serial_connection = serial.Serial(
                self.serial_port,
                self.serial_baudrate,
                timeout=0.1,
            )
            self.get_logger().info(
                'Serial forwarding enabled on %s at %d baud.'
                % (self.serial_port, self.serial_baudrate)
            )
        except Exception as exc:
            self.get_logger().warning(
                'Could not open serial port %s: %s'
                % (self.serial_port, exc)
            )

    def control_loop(self):
        """Read keyboard, publish command, and forward to serial."""
        self.process_keyboard()

        steering_delta_limit = self.steering_rate_limit / max(
            self.publish_rate_hz,
            1.0,
        )
        self.applied_steering = move_towards(
            self.applied_steering,
            self.current_angular,
            steering_delta_limit,
        )

        command = Twist()
        command.linear.x = self.current_linear
        command.angular.z = steering_to_yaw_rate(
            self.current_linear,
            self.applied_steering,
            self.wheel_base,
            self.max_turn_rate,
        )
        self.command_publisher.publish(command)
        self.forward_serial_command(
            self.current_linear,
            self.applied_steering,
        )

        if self.shutdown_requested:
            rclpy.shutdown()

    def joint_state_callback(self, msg):
        """Update the averaged steering angle from joint states."""
        joint_positions = dict(zip(msg.name, msg.position))
        steering_values = []

        if self.left_steering_joint_name in joint_positions:
            steering_values.append(joint_positions[self.left_steering_joint_name])
        if self.right_steering_joint_name in joint_positions:
            steering_values.append(joint_positions[self.right_steering_joint_name])

        if steering_values:
            self.current_steering_angle = sum(steering_values) / len(steering_values)

    def process_keyboard(self):
        """Process all pending keystrokes and update commands."""
        if self.terminal_settings is None:
            return

        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = self.read_key_sequence()
            handled = True

            if key == '\x1b[A':
                self.current_linear = clamp(
                    self.current_linear + self.linear_step,
                    -self.max_speed,
                    self.max_speed,
                )
            elif key == '\x1b[B':
                self.current_linear = clamp(
                    self.current_linear - self.linear_step,
                    -self.max_speed,
                    self.max_speed,
                )
            elif key == '\x1b[C':
                self.current_angular = clamp(
                    self.current_angular - self.steering_step,
                    -self.max_turn_rate,
                    self.max_turn_rate,
                )
            elif key == '\x1b[D':
                self.current_angular = clamp(
                    self.current_angular + self.steering_step,
                    -self.max_turn_rate,
                    self.max_turn_rate,
                )
            elif key == ' ':
                self.current_linear = 0.0
                self.current_angular = 0.0
            elif key in ('q', 'Q'):
                self.current_linear = 0.0
                self.current_angular = 0.0
                self.shutdown_requested = True
            else:
                handled = False

            if handled:
                self.get_logger().info(
                    'command | steering=%.2f rad, speed=%.2f m/s'
                    % (self.current_angular, self.current_linear)
                )

    def read_key_sequence(self):
        """Read a single key or escape sequence from stdin."""
        first_character = sys.stdin.read(1)
        if first_character != '\x1b':
            return first_character

        sequence = first_character
        for _ in range(2):
            if select.select([sys.stdin], [], [], 0.0)[0]:
                sequence += sys.stdin.read(1)
        return sequence

    def forward_serial_command(self, speed, steering_angle):
        """Send the current speed and steering-angle intent to the Arduino."""
        if self.serial_connection is None:
            return

        payload = 'SPD:{:.3f},STR:{:.3f}\n'.format(
            speed,
            steering_angle,
        )
        try:
            self.serial_connection.write(payload.encode('ascii'))
        except Exception as exc:
            self.get_logger().warning('Serial forwarding failed: %s' % exc)
            self.serial_connection = None

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
        yaw_deg = math.degrees(quaternion_to_yaw(msg.pose.pose.orientation))

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
        """Send a stop command and restore the terminal."""
        if rclpy.ok():
            self.command_publisher.publish(Twist())
        if self.serial_connection is not None:
            try:
                self.serial_connection.write(b'SPD:0.000,STR:0.000\n')
                self.serial_connection.close()
            except Exception:
                pass
        if self.terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
        if rclpy.ok():
            self.get_logger().info('Vehicle stop command sent.')


def main(args=None):
    """Entry point for the teleoperation node."""
    rclpy.init(args=args)
    node = AutonomousSystemsMS2TeleopTeam23()
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
