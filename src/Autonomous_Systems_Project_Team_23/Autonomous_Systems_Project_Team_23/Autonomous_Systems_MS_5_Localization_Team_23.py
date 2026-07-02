"""Milestone 5 simulation localization node for Team 23.

This node sits between Gazebo's ideal odometry and the existing MS3/MS4
controller stack. It injects white Gaussian noise into the ideal state/input
stream, then applies a linearized discrete Kalman filter to publish filtered
odometry for the controllers and planner.
"""

import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as pyplot
from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.node import Node


def normalize_angle(angle_rad):
    """Normalize *angle_rad* to the [-pi, pi] interval."""
    wrapped = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def quaternion_to_yaw(quaternion):
    """Convert a geometry_msgs quaternion to a yaw angle in radians."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw_rad):
    """Return the planar z/w quaternion components for *yaw_rad*."""
    half_yaw = 0.5 * yaw_rad
    return math.sin(half_yaw), math.cos(half_yaw)


def stamp_to_seconds(stamp_msg):
    """Convert a builtin_interfaces/Time stamp to float seconds."""
    return float(stamp_msg.sec) + float(stamp_msg.nanosec) / 1_000_000_000.0


class AutonomousSystemsMS5LocalizationTeam23(Node):
    """Inject measurement noise and publish filtered odometry."""

    def __init__(self):
        super().__init__('autonomous_systems_ms_5_localization_team_23')

        self.declare_parameter('ideal_state_topic', '/ms5/ideal_odom')
        self.declare_parameter('filtered_state_topic', '/odom')
        self.declare_parameter('noisy_state_topic', '/ms5/noisy_odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('measurement_x_std', 0.03)
        self.declare_parameter('measurement_y_std', 0.03)
        self.declare_parameter('measurement_heading_std', 0.04)
        self.declare_parameter('measurement_speed_std', 0.05)
        self.declare_parameter('measurement_yaw_rate_std', 0.08)
        self.declare_parameter('process_speed_std', 0.025)
        self.declare_parameter('process_yaw_rate_std', 0.05)
        self.declare_parameter('initial_position_covariance', 0.05)
        self.declare_parameter('initial_heading_covariance', 0.05)
        self.declare_parameter('initial_speed_covariance', 0.08)
        self.declare_parameter('initial_yaw_rate_covariance', 0.08)
        self.declare_parameter('publish_noisy_state', True)
        self.declare_parameter('save_artifacts', True)
        self.declare_parameter(
            'artifact_output_dir',
            os.path.join(os.path.expanduser('~'), '.ros', 'team23_ms5'),
        )
        self.declare_parameter('artifact_stem', 'team23_ms5')
        self.declare_parameter('random_seed', 23)
        self.declare_parameter('max_history_points', 10000)

        self.ideal_state_topic = self.get_parameter('ideal_state_topic').value
        self.filtered_state_topic = self.get_parameter(
            'filtered_state_topic'
        ).value
        self.noisy_state_topic = self.get_parameter('noisy_state_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.measurement_x_std = float(
            self.get_parameter('measurement_x_std').value
        )
        self.measurement_y_std = float(
            self.get_parameter('measurement_y_std').value
        )
        self.measurement_heading_std = float(
            self.get_parameter('measurement_heading_std').value
        )
        self.measurement_speed_std = float(
            self.get_parameter('measurement_speed_std').value
        )
        self.measurement_yaw_rate_std = float(
            self.get_parameter('measurement_yaw_rate_std').value
        )
        self.process_speed_std = float(
            self.get_parameter('process_speed_std').value
        )
        self.process_yaw_rate_std = float(
            self.get_parameter('process_yaw_rate_std').value
        )
        self.initial_position_covariance = float(
            self.get_parameter('initial_position_covariance').value
        )
        self.initial_heading_covariance = float(
            self.get_parameter('initial_heading_covariance').value
        )
        self.initial_speed_covariance = float(
            self.get_parameter('initial_speed_covariance').value
        )
        self.initial_yaw_rate_covariance = float(
            self.get_parameter('initial_yaw_rate_covariance').value
        )
        self.publish_noisy_state = bool(
            self.get_parameter('publish_noisy_state').value
        )
        self.save_artifacts = bool(self.get_parameter('save_artifacts').value)
        self.artifact_output_dir = self.get_parameter('artifact_output_dir').value
        self.artifact_stem = self.get_parameter('artifact_stem').value
        self.max_history_points = int(self.get_parameter('max_history_points').value)

        random_seed = int(self.get_parameter('random_seed').value)
        self.random_generator = numpy.random.default_rng(random_seed)

        self.measurement_covariance = numpy.diag([
            self.measurement_x_std ** 2,
            self.measurement_y_std ** 2,
            self.measurement_heading_std ** 2,
            self.measurement_speed_std ** 2,
            self.measurement_yaw_rate_std ** 2,
        ])
        self.process_covariance = numpy.diag([
            1e-8,
            1e-8,
            1e-8,
            self.process_speed_std ** 2,
            self.process_yaw_rate_std ** 2,
        ])
        self.measurement_matrix = numpy.eye(5)
        self.identity_matrix = numpy.eye(5)

        self.filtered_state_publisher = self.create_publisher(
            Odometry,
            self.filtered_state_topic,
            10,
        )
        self.noisy_state_publisher = None
        if self.publish_noisy_state:
            self.noisy_state_publisher = self.create_publisher(
                Odometry,
                self.noisy_state_topic,
                10,
            )

        self.ideal_state_subscription = self.create_subscription(
            Odometry,
            self.ideal_state_topic,
            self.ideal_state_callback,
            10,
        )

        self.state_estimate = None
        self.state_covariance = None
        self.last_timestamp_sec = None
        self.start_timestamp_sec = None
        self.last_log_time = self.get_clock().now()
        self.history_rows = []

        self.get_logger().info(
            'MS5 localization ready. ideal_state_topic=%s, filtered_state_topic=%s, '
            'noisy_state_topic=%s.'
            % (
                self.ideal_state_topic,
                self.filtered_state_topic,
                self.noisy_state_topic,
            )
        )

    def ideal_state_callback(self, msg):
        """Process the latest ideal odometry from Gazebo."""
        timestamp_sec = stamp_to_seconds(msg.header.stamp)
        if timestamp_sec <= 0.0:
            timestamp_sec = (
                self.get_clock().now().nanoseconds / 1_000_000_000.0
            )

        if self.start_timestamp_sec is None:
            self.start_timestamp_sec = timestamp_sec

        ideal_state = self.extract_state_vector(msg)
        noisy_measurement = self.sample_noisy_measurement(ideal_state)

        if self.state_estimate is None or self.last_timestamp_sec is None:
            self.initialize_filter(noisy_measurement)
            filtered_state = self.state_estimate.copy()
        else:
            dt = timestamp_sec - self.last_timestamp_sec
            if dt <= 0.0 or dt > 1.0:
                dt = 1.0 / 20.0
            filtered_state = self.run_predict_update(dt, noisy_measurement)

        self.last_timestamp_sec = timestamp_sec
        self.publish_state(msg, filtered_state, self.filtered_state_publisher)
        if self.noisy_state_publisher is not None:
            self.publish_state(msg, noisy_measurement, self.noisy_state_publisher)
        self.append_history(timestamp_sec, ideal_state, noisy_measurement, filtered_state)
        self.log_state(filtered_state)

    def extract_state_vector(self, odom_msg):
        """Return [x, y, theta, v, omega] from an Odometry message."""
        return numpy.array([
            float(odom_msg.pose.pose.position.x),
            float(odom_msg.pose.pose.position.y),
            quaternion_to_yaw(odom_msg.pose.pose.orientation),
            float(odom_msg.twist.twist.linear.x),
            float(odom_msg.twist.twist.angular.z),
        ])

    def sample_noisy_measurement(self, ideal_state):
        """Add white Gaussian noise to the ideal state and input stream."""
        noisy = ideal_state.copy()
        noisy[0] += self.random_generator.normal(0.0, self.measurement_x_std)
        noisy[1] += self.random_generator.normal(0.0, self.measurement_y_std)
        noisy[2] = normalize_angle(
            noisy[2] + self.random_generator.normal(0.0, self.measurement_heading_std)
        )
        noisy[3] += self.random_generator.normal(0.0, self.measurement_speed_std)
        noisy[4] += self.random_generator.normal(0.0, self.measurement_yaw_rate_std)
        return noisy

    def initialize_filter(self, noisy_measurement):
        """Initialize the state estimate and covariance from the first sample."""
        self.state_estimate = noisy_measurement.copy()
        self.state_covariance = numpy.diag([
            self.initial_position_covariance ** 2,
            self.initial_position_covariance ** 2,
            self.initial_heading_covariance ** 2,
            self.initial_speed_covariance ** 2,
            self.initial_yaw_rate_covariance ** 2,
        ])

    def run_predict_update(self, dt, noisy_measurement):
        """Apply a linearized discrete Kalman filter step.

        The state is [x, y, theta, v, omega]. We predict forward using
        the discrete kinematic model, linearize around the current state,
        and then correct with the noisy measurement.
        """
        x_pos, y_pos, yaw_rad, speed, yaw_rate = self.state_estimate

        predicted_state = numpy.array([
            x_pos + dt * speed * math.cos(yaw_rad),
            y_pos + dt * speed * math.sin(yaw_rad),
            normalize_angle(yaw_rad + dt * yaw_rate),
            speed,
            yaw_rate,
        ])

        transition_jacobian = numpy.array([
            [1.0, 0.0, -dt * speed * math.sin(yaw_rad), dt * math.cos(yaw_rad), 0.0],
            [0.0, 1.0, dt * speed * math.cos(yaw_rad), dt * math.sin(yaw_rad), 0.0],
            [0.0, 0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0],
        ])

        predicted_covariance = (
            transition_jacobian
            @ self.state_covariance
            @ transition_jacobian.T
            + self.process_covariance
        )

        innovation = noisy_measurement - predicted_state
        innovation[2] = normalize_angle(innovation[2])
        innovation_covariance = (
            self.measurement_matrix
            @ predicted_covariance
            @ self.measurement_matrix.T
            + self.measurement_covariance
        )
        kalman_gain = (
            predicted_covariance
            @ self.measurement_matrix.T
            @ numpy.linalg.inv(innovation_covariance)
        )

        corrected_state = predicted_state + kalman_gain @ innovation
        corrected_state[2] = normalize_angle(corrected_state[2])
        corrected_covariance = (
            self.identity_matrix - kalman_gain @ self.measurement_matrix
        ) @ predicted_covariance

        self.state_estimate = corrected_state
        self.state_covariance = corrected_covariance
        return corrected_state.copy()

    def publish_state(self, template_msg, state_vector, publisher):
        """Publish a state vector as nav_msgs/Odometry."""
        odom = Odometry()
        odom.header = template_msg.header
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = float(state_vector[0])
        odom.pose.pose.position.y = float(state_vector[1])
        odom.pose.pose.position.z = 0.0
        quaternion_z, quaternion_w = yaw_to_quaternion(float(state_vector[2]))
        odom.pose.pose.orientation.z = quaternion_z
        odom.pose.pose.orientation.w = quaternion_w
        odom.twist.twist.linear.x = float(state_vector[3])
        odom.twist.twist.angular.z = float(state_vector[4])

        if self.state_covariance is not None:
            odom.pose.covariance[0] = float(self.state_covariance[0, 0])
            odom.pose.covariance[7] = float(self.state_covariance[1, 1])
            odom.pose.covariance[35] = float(self.state_covariance[2, 2])
            odom.twist.covariance[0] = float(self.state_covariance[3, 3])
            odom.twist.covariance[35] = float(self.state_covariance[4, 4])

        publisher.publish(odom)

    def append_history(self, timestamp_sec, ideal_state, noisy_state, filtered_state):
        """Store a bounded history used for CSV export and plotting."""
        if len(self.history_rows) >= self.max_history_points:
            self.history_rows.pop(0)

        self.history_rows.append({
            'time_sec': timestamp_sec - self.start_timestamp_sec,
            'ideal_x': float(ideal_state[0]),
            'ideal_y': float(ideal_state[1]),
            'ideal_theta': float(ideal_state[2]),
            'ideal_speed': float(ideal_state[3]),
            'ideal_yaw_rate': float(ideal_state[4]),
            'noisy_x': float(noisy_state[0]),
            'noisy_y': float(noisy_state[1]),
            'noisy_theta': float(noisy_state[2]),
            'noisy_speed': float(noisy_state[3]),
            'noisy_yaw_rate': float(noisy_state[4]),
            'filtered_x': float(filtered_state[0]),
            'filtered_y': float(filtered_state[1]),
            'filtered_theta': float(filtered_state[2]),
            'filtered_speed': float(filtered_state[3]),
            'filtered_yaw_rate': float(filtered_state[4]),
        })

    def log_state(self, filtered_state):
        """Throttle terminal output for live validation."""
        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1_000_000_000.0
        if elapsed < 0.5:
            return

        self.get_logger().info(
            'ms5 localization | x=%.2f m, y=%.2f m, heading=%.2f deg, '
            'speed=%.2f m/s, yaw_rate=%.2f rad/s'
            % (
                filtered_state[0],
                filtered_state[1],
                math.degrees(filtered_state[2]),
                filtered_state[3],
                filtered_state[4],
            )
        )
        self.last_log_time = now

    def save_artifacts_to_disk(self):
        """Write CSV and PNG outputs for the MS5 graph deliverable."""
        if not self.save_artifacts or not self.history_rows:
            return

        os.makedirs(self.artifact_output_dir, exist_ok=True)
        csv_path = os.path.join(
            self.artifact_output_dir,
            f'{self.artifact_stem}_history.csv',
        )
        overview_path = os.path.join(
            self.artifact_output_dir,
            f'{self.artifact_stem}_overview.png',
        )
        path_path = os.path.join(
            self.artifact_output_dir,
            f'{self.artifact_stem}_xy_path.png',
        )

        with open(csv_path, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=self.history_rows[0].keys())
            writer.writeheader()
            writer.writerows(self.history_rows)

        times = [row['time_sec'] for row in self.history_rows]
        figure, axes = pyplot.subplots(3, 2, figsize=(14, 10), sharex='col')
        self.plot_signal(axes[0, 0], times, 'x (m)', 'ideal_x', 'noisy_x', 'filtered_x')
        self.plot_signal(axes[1, 0], times, 'y (m)', 'ideal_y', 'noisy_y', 'filtered_y')
        self.plot_signal(
            axes[2, 0],
            times,
            'heading (deg)',
            'ideal_theta',
            'noisy_theta',
            'filtered_theta',
            angle_plot=True,
        )
        self.plot_signal(
            axes[0, 1],
            times,
            'speed (m/s)',
            'ideal_speed',
            'noisy_speed',
            'filtered_speed',
        )
        self.plot_signal(
            axes[1, 1],
            times,
            'yaw rate (rad/s)',
            'ideal_yaw_rate',
            'noisy_yaw_rate',
            'filtered_yaw_rate',
        )
        axes[2, 1].axis('off')
        axes[2, 1].text(
            0.0,
            0.8,
            'Noise configuration\n'
            f'x std = {self.measurement_x_std:.3f} m\n'
            f'y std = {self.measurement_y_std:.3f} m\n'
            f'heading std = {self.measurement_heading_std:.3f} rad\n'
            f'speed std = {self.measurement_speed_std:.3f} m/s\n'
            f'yaw-rate std = {self.measurement_yaw_rate_std:.3f} rad/s',
            fontsize=10,
            va='top',
        )
        axes[2, 0].set_xlabel('time (s)')
        axes[1, 1].set_xlabel('time (s)')
        figure.tight_layout()
        figure.savefig(overview_path, dpi=150)
        pyplot.close(figure)

        path_figure, path_axis = pyplot.subplots(figsize=(8, 6))
        path_axis.plot(
            [row['ideal_x'] for row in self.history_rows],
            [row['ideal_y'] for row in self.history_rows],
            label='ideal',
        )
        path_axis.plot(
            [row['noisy_x'] for row in self.history_rows],
            [row['noisy_y'] for row in self.history_rows],
            label='noisy',
            alpha=0.6,
        )
        path_axis.plot(
            [row['filtered_x'] for row in self.history_rows],
            [row['filtered_y'] for row in self.history_rows],
            label='filtered',
            linewidth=2.0,
        )
        path_axis.set_title('MS5 filtered vs noisy path')
        path_axis.set_xlabel('x (m)')
        path_axis.set_ylabel('y (m)')
        path_axis.axis('equal')
        path_axis.grid(True)
        path_axis.legend()
        path_figure.tight_layout()
        path_figure.savefig(path_path, dpi=150)
        pyplot.close(path_figure)

        self.get_logger().info(
            'MS5 artifacts saved to %s' % self.artifact_output_dir
        )

    def plot_signal(
        self,
        axis,
        times,
        ylabel,
        ideal_key,
        noisy_key,
        filtered_key,
        angle_plot=False,
    ):
        """Plot ideal/noisy/filtered signals on the provided axis."""
        def series(key):
            values = [row[key] for row in self.history_rows]
            if angle_plot:
                return [math.degrees(value) for value in values]
            return values

        axis.plot(times, series(ideal_key), label='ideal')
        axis.plot(times, series(noisy_key), label='noisy', alpha=0.65)
        axis.plot(times, series(filtered_key), label='filtered', linewidth=2.0)
        axis.set_ylabel(ylabel)
        axis.grid(True)
        axis.legend(loc='best')

    def stop_node(self):
        """Persist plots/CSV at shutdown."""
        self.save_artifacts_to_disk()


def main(args=None):
    """Entry point for the MS5 localization node."""
    rclpy.init(args=args)
    node = AutonomousSystemsMS5LocalizationTeam23()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
