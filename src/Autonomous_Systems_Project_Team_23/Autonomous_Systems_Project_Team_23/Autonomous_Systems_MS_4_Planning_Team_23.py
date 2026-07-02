"""Milestone 4 planning node for Team 23."""

import bisect
import math
import os
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def quaternion_to_yaw(quaternion):
    """Convert a quaternion to a yaw angle in radians."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, minimum, maximum):
    """Clamp *value* to the inclusive [minimum, maximum] interval."""
    return max(minimum, min(maximum, value))


def lerp(start_value, end_value, progress):
    """Linearly interpolate between two values."""
    return start_value + (end_value - start_value) * progress


def smoothstep(progress):
    """Ease a 0..1 transition to avoid abrupt lane targets."""
    progress = clamp(progress, 0.0, 1.0)
    return progress * progress * (3.0 - 2.0 * progress)


def smoothstep_derivative(progress):
    """Derivative of smoothstep for heading continuity."""
    progress = clamp(progress, 0.0, 1.0)
    return 6.0 * progress * (1.0 - progress)


def quintic_blend(progress):
    """5th-order blend with zero slope and curvature at both ends."""
    progress = clamp(progress, 0.0, 1.0)
    return (
        10.0 * progress ** 3
        - 15.0 * progress ** 4
        + 6.0 * progress ** 5
    )


def quintic_blend_derivative(progress):
    """Derivative of :func:`quintic_blend` with respect to progress."""
    progress = clamp(progress, 0.0, 1.0)
    return (
        30.0 * progress ** 2
        - 60.0 * progress ** 3
        + 30.0 * progress ** 4
    )


def path_normal_coordinate(x_position, y_position, path_heading):
    """Project a position onto the path normal for a given heading."""
    return (
        -math.sin(path_heading) * x_position
        + math.cos(path_heading) * y_position
    )


def _safe_float(text):
    if text is None:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def normalize_angle(angle_rad):
    """Normalize angle to [-pi, pi]."""
    wrapped = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def normalize_track_mode(track_mode):
    """Map launch aliases to the internal planner track identifiers."""
    normalized = str(track_mode).strip().lower()
    aliases = {
        'track_01': 'empty_track',
        'track_1': 'empty_track',
        'empty': 'empty_track',
        'empty_track': 'empty_track',
        'track_02': 'racing_track',
        'track_2': 'racing_track',
        'racing': 'racing_track',
        'racing_track': 'racing_track',
        'track_03': 'city_track',
        'track_3': 'city_track',
        'city': 'city_track',
        'city_track': 'city_track',
    }
    return aliases.get(normalized, 'empty_track')


class AutonomousSystemsMS4PlanningTeam23(Node):
    """Publish desired speed, path offset, and heading for MS4."""

    def __init__(self):
        super().__init__('autonomous_systems_ms_4_planning_team_23')

        self.declare_parameter('state_topic', '/odom')
        self.declare_parameter('desired_speed_topic', '/ms4/desired_speed')
        self.declare_parameter('desired_lane_topic', '/ms4/desired_lane')
        self.declare_parameter('desired_heading_topic', '/ms4/desired_heading')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('planner_track_mode', 'empty_track')
        self.declare_parameter('startup_settle_samples', 12)
        self.declare_parameter('startup_max_position_jump', 0.25)
        self.declare_parameter('startup_max_heading_jump_deg', 20.0)
        self.declare_parameter('startup_log_wait_seconds', 1.0)

        self.declare_parameter('empty_track_speed', 0.50)
        self.declare_parameter('empty_track_align_speed', 0.18)
        self.declare_parameter('empty_track_lane', 0.0)
        self.declare_parameter('empty_track_lane_second', 0.0)
        self.declare_parameter('empty_track_lane_second_start', 1e9)
        self.declare_parameter('empty_track_lane_second_transition_distance', 0.0)
        self.declare_parameter('empty_track_heading', 0.0)
        self.declare_parameter('empty_track_length', 10.0)
        self.declare_parameter('empty_track_lane_transition_distance', 0.0)
        self.declare_parameter('empty_track_vehicle_length', 0.39)
        self.declare_parameter('empty_track_vehicle_width', 0.18)
        self.declare_parameter('empty_track_wall_margin', 0.08)
        self.declare_parameter('empty_track_align_lateral_tolerance', 0.02)
        self.declare_parameter('empty_track_align_heading_tolerance_deg', 3.0)
        self.declare_parameter('empty_track_stop_position_tolerance', 0.02)

        self.declare_parameter('racing_track_length', 10.0)
        self.declare_parameter('racing_lane_left', 0.1875)
        self.declare_parameter('racing_lane_right', -0.1875)
        self.declare_parameter('racing_obstacle_1_x', 4.0)
        self.declare_parameter('racing_obstacle_2_x', 8.0)
        self.declare_parameter('racing_lane_change_buffer', 2.8)
        self.declare_parameter('racing_lane_change_end_margin', 0.50)
        self.declare_parameter('racing_lane_settle_tolerance', 0.05)
        self.declare_parameter('racing_straight_speed', 0.26)
        self.declare_parameter('racing_lane_change_speed', 0.14)
        self.declare_parameter('racing_vehicle_length', 0.39)
        self.declare_parameter('racing_vehicle_width', 0.18)
        self.declare_parameter('racing_wall_margin', 0.08)
        self.declare_parameter('racing_obstacle_longitudinal_margin', 0.05)
        self.declare_parameter('racing_lookahead', 0.35)
        self.declare_parameter('racing_path_resolution', 0.05)
        self.declare_parameter('racing_search_window', 60)
        self.declare_parameter('racing_world_file', 'Racing_Track.world')
        self.declare_parameter('racing_clearance', 0.02)
        self.declare_parameter('racing_heading_threshold', 0.05)

        # Optional fixed lane profile override for the racing track.
        # This is useful when obstacle positions are known and the track is straight.
        self.declare_parameter('racing_override_lane_profile', True)
        self.declare_parameter('racing_profile_lane_1', 0.1875)
        self.declare_parameter('racing_profile_lane_2', -0.1875)
        self.declare_parameter('racing_profile_lane_3', 0.1875)
        self.declare_parameter('racing_profile_switch_1_start', 1.6)
        self.declare_parameter('racing_profile_switch_1_distance', 2.0)
        self.declare_parameter('racing_profile_switch_2_start', 5.6)
        self.declare_parameter('racing_profile_switch_2_distance', 2.0)

        self.declare_parameter('city_min_x', -2.25)
        self.declare_parameter('city_max_x', 2.25)
        self.declare_parameter('city_min_y', -3.5)
        self.declare_parameter('city_max_y', 0.0)
        self.declare_parameter('city_corner_margin', 0.65)
        self.declare_parameter('city_straight_speed', 0.45)
        self.declare_parameter('city_turn_speed', 0.25)
        self.declare_parameter('city_corner_radius', 0.75)
        self.declare_parameter('city_lookahead', 0.45)
        self.declare_parameter('city_speed_blend_distance', 0.35)
        self.declare_parameter('city_path_resolution', 0.05)
        self.declare_parameter('city_search_window', 80)
        self.declare_parameter('city_world_file', 'City_Track.world')
        self.declare_parameter('city_curvature_threshold', 0.6)

        default_output_dir = os.path.join(
            os.path.expanduser('~'),
            '.ros',
            'team23_ms4_paths',
        )
        self.declare_parameter('save_centerline', True)
        self.declare_parameter('centerline_output_dir', default_output_dir)
        self.declare_parameter('centerline_overwrite', False)

        self.state_topic = self.get_parameter('state_topic').value
        self.desired_speed_topic = self.get_parameter(
            'desired_speed_topic'
        ).value
        self.desired_lane_topic = self.get_parameter('desired_lane_topic').value
        self.desired_heading_topic = self.get_parameter(
            'desired_heading_topic'
        ).value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.planner_track_mode = normalize_track_mode(
            self.get_parameter('planner_track_mode').value
        )
        self.startup_settle_samples = int(
            self.get_parameter('startup_settle_samples').value
        )
        self.startup_max_position_jump = float(
            self.get_parameter('startup_max_position_jump').value
        )
        self.startup_max_heading_jump_deg = float(
            self.get_parameter('startup_max_heading_jump_deg').value
        )
        self.startup_log_wait_seconds = float(
            self.get_parameter('startup_log_wait_seconds').value
        )

        self.empty_track_speed = float(
            self.get_parameter('empty_track_speed').value
        )
        self.empty_track_align_speed = float(
            self.get_parameter('empty_track_align_speed').value
        )
        self.empty_track_lane = float(self.get_parameter('empty_track_lane').value)
        self.empty_track_lane_second = float(
            self.get_parameter('empty_track_lane_second').value
        )
        self.empty_track_lane_second_start = float(
            self.get_parameter('empty_track_lane_second_start').value
        )
        self.empty_track_lane_second_transition_distance = float(
            self.get_parameter('empty_track_lane_second_transition_distance').value
        )
        self.empty_track_heading = float(
            self.get_parameter('empty_track_heading').value
        )
        self.empty_track_length = float(
            self.get_parameter('empty_track_length').value
        )
        self.empty_track_lane_transition_distance = float(
            self.get_parameter('empty_track_lane_transition_distance').value
        )
        self.empty_track_vehicle_length = float(
            self.get_parameter('empty_track_vehicle_length').value
        )
        self.empty_track_vehicle_width = float(
            self.get_parameter('empty_track_vehicle_width').value
        )
        self.empty_track_wall_margin = float(
            self.get_parameter('empty_track_wall_margin').value
        )
        self.empty_track_align_lateral_tolerance = float(
            self.get_parameter('empty_track_align_lateral_tolerance').value
        )
        self.empty_track_align_heading_tolerance_deg = float(
            self.get_parameter('empty_track_align_heading_tolerance_deg').value
        )
        self.empty_track_stop_position_tolerance = float(
            self.get_parameter('empty_track_stop_position_tolerance').value
        )

        self.racing_track_length = float(
            self.get_parameter('racing_track_length').value
        )
        self.racing_lane_left = float(
            self.get_parameter('racing_lane_left').value
        )
        self.racing_lane_right = float(
            self.get_parameter('racing_lane_right').value
        )
        self.racing_obstacle_1_x = float(
            self.get_parameter('racing_obstacle_1_x').value
        )
        self.racing_obstacle_2_x = float(
            self.get_parameter('racing_obstacle_2_x').value
        )
        self.racing_lane_change_buffer = float(
            self.get_parameter('racing_lane_change_buffer').value
        )
        self.racing_lane_change_end_margin = float(
            self.get_parameter('racing_lane_change_end_margin').value
        )
        self.racing_lane_settle_tolerance = float(
            self.get_parameter('racing_lane_settle_tolerance').value
        )
        self.racing_straight_speed = float(
            self.get_parameter('racing_straight_speed').value
        )
        self.racing_lane_change_speed = float(
            self.get_parameter('racing_lane_change_speed').value
        )
        self.racing_vehicle_length = float(
            self.get_parameter('racing_vehicle_length').value
        )
        self.racing_vehicle_width = float(
            self.get_parameter('racing_vehicle_width').value
        )
        self.racing_wall_margin = float(
            self.get_parameter('racing_wall_margin').value
        )
        self.racing_obstacle_longitudinal_margin = float(
            self.get_parameter('racing_obstacle_longitudinal_margin').value
        )
        self.racing_lookahead = float(
            self.get_parameter('racing_lookahead').value
        )
        self.racing_path_resolution = float(
            self.get_parameter('racing_path_resolution').value
        )
        self.racing_search_window = int(
            self.get_parameter('racing_search_window').value
        )
        self.racing_world_file = str(
            self.get_parameter('racing_world_file').value
        )
        self.racing_clearance = float(
            self.get_parameter('racing_clearance').value
        )
        self.racing_heading_threshold = float(
            self.get_parameter('racing_heading_threshold').value
        )

        self.racing_override_lane_profile = bool(
            self.get_parameter('racing_override_lane_profile').value
        )
        self.racing_profile_lane_1 = float(
            self.get_parameter('racing_profile_lane_1').value
        )
        self.racing_profile_lane_2 = float(
            self.get_parameter('racing_profile_lane_2').value
        )
        self.racing_profile_lane_3 = float(
            self.get_parameter('racing_profile_lane_3').value
        )
        self.racing_profile_switch_1_start = float(
            self.get_parameter('racing_profile_switch_1_start').value
        )
        self.racing_profile_switch_1_distance = float(
            self.get_parameter('racing_profile_switch_1_distance').value
        )
        self.racing_profile_switch_2_start = float(
            self.get_parameter('racing_profile_switch_2_start').value
        )
        self.racing_profile_switch_2_distance = float(
            self.get_parameter('racing_profile_switch_2_distance').value
        )

        self.city_min_x = float(self.get_parameter('city_min_x').value)
        self.city_max_x = float(self.get_parameter('city_max_x').value)
        self.city_min_y = float(self.get_parameter('city_min_y').value)
        self.city_max_y = float(self.get_parameter('city_max_y').value)
        self.city_corner_margin = float(
            self.get_parameter('city_corner_margin').value
        )
        self.city_straight_speed = float(
            self.get_parameter('city_straight_speed').value
        )
        self.city_turn_speed = float(
            self.get_parameter('city_turn_speed').value
        )
        self.city_corner_radius = float(
            self.get_parameter('city_corner_radius').value
        )
        self.city_lookahead = float(
            self.get_parameter('city_lookahead').value
        )
        self.city_speed_blend_distance = float(
            self.get_parameter('city_speed_blend_distance').value
        )
        self.city_path_resolution = float(
            self.get_parameter('city_path_resolution').value
        )
        self.city_search_window = int(
            self.get_parameter('city_search_window').value
        )
        self.city_world_file = str(
            self.get_parameter('city_world_file').value
        )
        self.city_curvature_threshold = float(
            self.get_parameter('city_curvature_threshold').value
        )
        self.save_centerline = bool(
            self.get_parameter('save_centerline').value
        )
        self.centerline_output_dir = str(
            self.get_parameter('centerline_output_dir').value
        )
        self.centerline_overwrite = bool(
            self.get_parameter('centerline_overwrite').value
        )

        self.speed_publisher = self.create_publisher(
            Float64,
            self.desired_speed_topic,
            10,
        )
        self.lane_publisher = self.create_publisher(
            Float64,
            self.desired_lane_topic,
            10,
        )
        self.heading_publisher = self.create_publisher(
            Float64,
            self.desired_heading_topic,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.state_topic,
            self.odom_callback,
            10,
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.last_log_time = self.get_clock().now()
        self.startup_last_wait_log_time = self.get_clock().now()
        self.simulation_state_ready = False
        self.startup_good_sample_count = 0
        self.previous_odom_x = None
        self.previous_odom_y = None
        self.previous_odom_yaw = None
        self.empty_track_stop_latched = False
        self.empty_track_has_valid_pose = False
        self.empty_track_last_wall_clearance = 999.0
        self.empty_track_last_state = 'waiting_for_valid_pose'
        self.racing_last_index = 0
        self.city_last_index = 0
        self.racing_last_wall_clearance = 999.0
        self.racing_last_obstacle_clearance = 999.0
        self.racing_obstacles = []
        self.racing_track_top_y = 0.375
        self.racing_track_bottom_y = -0.375
        self.racing_lane_plan = []

        self._build_city_path()
        self._build_racing_path_samples()
        self._build_city_path_samples()

        if self.save_centerline:
            self._persist_centerlines()

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.publish_timer = self.create_timer(period, self.publish_plan)

        self.get_logger().info(
            'MS4 planner ready. track_mode=%s, state_topic=%s.'
            % (
                self.planner_track_mode,
                self.state_topic,
            )
        )

    def odom_callback(self, msg):
        """Update the latest vehicle state from odometry."""
        x_pos = float(msg.pose.pose.position.x)
        y_pos = float(msg.pose.pose.position.y)
        yaw_rad = quaternion_to_yaw(msg.pose.pose.orientation)

        self.current_x = x_pos
        self.current_y = y_pos
        self.current_yaw = yaw_rad

        if self.simulation_state_ready:
            return

        if not self._odom_sample_is_plausible(x_pos, y_pos, yaw_rad):
            self.startup_good_sample_count = 0
            self.previous_odom_x = x_pos
            self.previous_odom_y = y_pos
            self.previous_odom_yaw = yaw_rad
            return

        if self.previous_odom_x is None:
            self.startup_good_sample_count = 1
        else:
            dx = x_pos - self.previous_odom_x
            dy = y_pos - self.previous_odom_y
            distance = math.hypot(dx, dy)
            yaw_jump_deg = abs(
                math.degrees(normalize_angle(yaw_rad - self.previous_odom_yaw))
            )
            if (
                distance <= self.startup_max_position_jump
                and yaw_jump_deg <= self.startup_max_heading_jump_deg
            ):
                self.startup_good_sample_count += 1
            else:
                self.startup_good_sample_count = 1

        self.previous_odom_x = x_pos
        self.previous_odom_y = y_pos
        self.previous_odom_yaw = yaw_rad

        if self.startup_good_sample_count >= self.startup_settle_samples:
            self.simulation_state_ready = True
            self.get_logger().info(
                'Planner startup gate satisfied after %d stable odometry samples.'
                % self.startup_good_sample_count
            )

    def publish_plan(self):
        """Publish the plan for the currently selected milestone track."""
        if not self.simulation_state_ready:
            self._publish_startup_hold()
            return

        desired_speed, desired_lane, desired_heading = self.compute_plan()

        speed_message = Float64()
        speed_message.data = desired_speed
        self.speed_publisher.publish(speed_message)

        lane_message = Float64()
        lane_message.data = desired_lane
        self.lane_publisher.publish(lane_message)

        heading_message = Float64()
        heading_message.data = desired_heading
        self.heading_publisher.publish(heading_message)

        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1_000_000_000.0
        if elapsed >= 0.5:
            if self.planner_track_mode == 'racing_track':
                self.get_logger().info(
                    'planner | track=%s, pose=(%.2f, %.2f, %.2f deg), '
                    'speed=%.2f m/s, lane=%.2f m, heading=%.2f deg, '
                    'wall_clear=%.3f m, obstacle_clear=%.3f m'
                    % (
                        self.planner_track_mode,
                        self.current_x,
                        self.current_y,
                        math.degrees(self.current_yaw),
                        desired_speed,
                        desired_lane,
                        math.degrees(desired_heading),
                        self.racing_last_wall_clearance,
                        self.racing_last_obstacle_clearance,
                    )
                )
            else:
                self.get_logger().info(
                    'planner | track=%s, pose=(%.2f, %.2f, %.2f deg), '
                    'speed=%.2f m/s, lane=%.2f m, heading=%.2f deg, '
                    'state=%s, wall_clear=%.3f m'
                    % (
                        self.planner_track_mode,
                        self.current_x,
                        self.current_y,
                        math.degrees(self.current_yaw),
                        desired_speed,
                        desired_lane,
                        math.degrees(desired_heading),
                        self.empty_track_last_state,
                        self.empty_track_last_wall_clearance,
                    )
                )
            self.last_log_time = now

    def _publish_startup_hold(self):
        """Hold the vehicle stationary until Gazebo odometry is stable."""
        speed_message = Float64()
        speed_message.data = 0.0
        self.speed_publisher.publish(speed_message)

        lane_message = Float64()
        lane_message.data = 0.0
        self.lane_publisher.publish(lane_message)

        heading_message = Float64()
        heading_message.data = 0.0
        self.heading_publisher.publish(heading_message)

        now = self.get_clock().now()
        elapsed = (
            now - self.startup_last_wait_log_time
        ).nanoseconds / 1_000_000_000.0
        if elapsed >= self.startup_log_wait_seconds:
            self.get_logger().info(
                'planner | waiting_for_valid_pose, stable_samples=%d/%d, '
                'pose=(%.2f, %.2f, %.2f deg)'
                % (
                    self.startup_good_sample_count,
                    self.startup_settle_samples,
                    self.current_x,
                    self.current_y,
                    math.degrees(self.current_yaw),
                )
            )
            self.startup_last_wait_log_time = now

    def _odom_sample_is_plausible(self, x_pos, y_pos, yaw_rad):
        """Reject obviously bogus startup odometry before planning begins."""
        if not math.isfinite(x_pos) or not math.isfinite(y_pos) or not math.isfinite(yaw_rad):
            return False

        if self.planner_track_mode == 'city_track':
            return -4.0 <= x_pos <= 4.0 and -5.0 <= y_pos <= 1.0

        return -1.0 <= x_pos <= (self.empty_track_length + 2.0) and abs(y_pos) <= 1.0

    def compute_plan(self):
        """Select the active planner branch."""
        if self.planner_track_mode == 'racing_track':
            return self.plan_racing_track()
        if self.planner_track_mode == 'city_track':
            return self.plan_city_track()
        return self.plan_empty_track()

    @staticmethod
    def _wrap_angle_positive(angle):
        two_pi = 2.0 * math.pi
        while angle < 0.0:
            angle += two_pi
        while angle >= two_pi:
            angle -= two_pi
        return angle

    def _build_city_path(self):
        radius = max(0.05, float(self.city_corner_radius))
        right_x = self.city_max_x
        left_x = self.city_min_x
        top_y = self.city_max_y
        bottom_y = self.city_min_y

        width = right_x - left_x
        height = top_y - bottom_y
        max_radius = 0.5 * min(width, height) - 1e-3
        if max_radius <= 0.0:
            self.city_path = []
            self.city_path_length = 0.0
            return

        radius = min(radius, max_radius)
        x_inner = right_x - radius
        y_top_inner = top_y - radius
        y_bottom_inner = bottom_y + radius
        top_len = width - 2.0 * radius
        right_len = height - 2.0 * radius

        if top_len <= 0.0 or right_len <= 0.0:
            self.city_path = []
            self.city_path_length = 0.0
            return

        arc_len = radius * math.pi / 2.0
        segments = [
            {
                'type': 'line',
                'start_x': left_x + radius,
                'start_y': top_y,
                'heading': 0.0,
                'length': top_len,
            },
            {
                'type': 'arc',
                'center_x': x_inner,
                'center_y': y_top_inner,
                'radius': radius,
                'start_angle': math.pi / 2.0,
                'direction': -1.0,
                'length': arc_len,
            },
            {
                'type': 'line',
                'start_x': right_x,
                'start_y': y_top_inner,
                'heading': -math.pi / 2.0,
                'length': right_len,
            },
            {
                'type': 'arc',
                'center_x': x_inner,
                'center_y': y_bottom_inner,
                'radius': radius,
                'start_angle': 0.0,
                'direction': -1.0,
                'length': arc_len,
            },
            {
                'type': 'line',
                'start_x': x_inner,
                'start_y': bottom_y,
                'heading': math.pi,
                'length': top_len,
            },
            {
                'type': 'arc',
                'center_x': left_x + radius,
                'center_y': y_bottom_inner,
                'radius': radius,
                'start_angle': -math.pi / 2.0,
                'direction': -1.0,
                'length': arc_len,
            },
            {
                'type': 'line',
                'start_x': left_x,
                'start_y': y_bottom_inner,
                'heading': math.pi / 2.0,
                'length': right_len,
            },
            {
                'type': 'arc',
                'center_x': left_x + radius,
                'center_y': y_top_inner,
                'radius': radius,
                'start_angle': math.pi,
                'direction': -1.0,
                'length': arc_len,
            },
        ]

        s_cursor = 0.0
        for segment in segments:
            segment['s_start'] = s_cursor
            if segment['type'] == 'arc':
                segment['span'] = segment['length'] / segment['radius']
            s_cursor += segment['length']

        self.city_path = segments
        self.city_path_length = s_cursor

    def _locate_world_file(self, world_file_name):
        package_name = 'Autonomous_Systems_Project_Team_23'
        pkg_share = get_package_share_directory(package_name)
        packaged = os.path.join(pkg_share, 'Worlds', world_file_name)
        if os.path.isfile(packaged):
            return packaged

        local = os.path.join(os.getcwd(), world_file_name)
        if os.path.isfile(local):
            return local

        return None

    def _iter_models_from_world(self, world_file_path):
        try:
            tree = ElementTree.parse(world_file_path)
        except (ElementTree.ParseError, FileNotFoundError, OSError):
            return []

        root = tree.getroot()
        if root.tag != 'sdf':
            root = root.find('sdf') or root
        world = root.find('world') if root is not None else None
        if world is None:
            return []
        return list(world.findall('model'))

    def _parse_wall_segments(self, world_file_path):
        models = self._iter_models_from_world(world_file_path)
        segments = []
        for model in models:
            model_name = model.attrib.get('name', '')
            for link in model.findall('link'):
                link_name = link.attrib.get('name', '')
                full_name = f'{model_name}:{link_name}' if link_name else model_name
                pose = link.find('pose')
                if pose is None or pose.text is None:
                    continue
                pose_values = [v for v in pose.text.strip().split() if v]
                if len(pose_values) < 6:
                    continue
                x_pos = _safe_float(pose_values[0])
                y_pos = _safe_float(pose_values[1])
                yaw = _safe_float(pose_values[5])
                if x_pos is None or y_pos is None or yaw is None:
                    continue

                box = link.find('visual/geometry/box')
                if box is None:
                    box = link.find('collision/geometry/box')
                if box is None:
                    continue
                size = box.find('size')
                if size is None or size.text is None:
                    continue
                size_values = [v for v in size.text.strip().split() if v]
                if len(size_values) < 2:
                    continue
                length = _safe_float(size_values[0])
                width = _safe_float(size_values[1])
                if length is None or width is None:
                    continue
                if length < width:
                    length, width = width, length

                half_length = 0.5 * length
                dx = math.cos(yaw) * half_length
                dy = math.sin(yaw) * half_length
                start = (x_pos - dx, y_pos - dy)
                end = (x_pos + dx, y_pos + dy)
                segments.append({
                    'name': full_name,
                    'start': start,
                    'end': end,
                    'center': (x_pos, y_pos),
                    'length': length,
                    'width': width,
                })
        return segments

    def _split_walls_by_orientation(self, segments):
        vertical = []
        horizontal = []
        for segment in segments:
            (x0, y0) = segment['start']
            (x1, y1) = segment['end']
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            if dx >= dy:
                horizontal.append(segment)
            else:
                vertical.append(segment)
        return horizontal, vertical

    def _build_city_path_from_world(self, world_file_path):
        segments = self._parse_wall_segments(world_file_path)
        if not segments:
            return False

        horizontal, vertical = self._split_walls_by_orientation(segments)
        if len(horizontal) < 2 or len(vertical) < 2:
            return False

        horizontal = sorted(horizontal, key=lambda seg: (seg['start'][1] + seg['end'][1]) * 0.5)
        vertical = sorted(vertical, key=lambda seg: (seg['start'][0] + seg['end'][0]) * 0.5)

        top_outer = horizontal[-1]
        top_inner = horizontal[-2]
        bottom_outer = horizontal[0]
        bottom_inner = horizontal[1]

        right_outer = vertical[-1]
        right_inner = vertical[-2]
        left_outer = vertical[0]
        left_inner = vertical[1]

        top_y = 0.5 * ((top_outer['start'][1] + top_outer['end'][1]) * 0.5
                       + (top_inner['start'][1] + top_inner['end'][1]) * 0.5)
        bottom_y = 0.5 * ((bottom_outer['start'][1] + bottom_outer['end'][1]) * 0.5
                          + (bottom_inner['start'][1] + bottom_inner['end'][1]) * 0.5)
        left_x = 0.5 * ((left_outer['start'][0] + left_outer['end'][0]) * 0.5
                        + (left_inner['start'][0] + left_inner['end'][0]) * 0.5)
        right_x = 0.5 * ((right_outer['start'][0] + right_outer['end'][0]) * 0.5
                         + (right_inner['start'][0] + right_inner['end'][0]) * 0.5)

        top_outer_width = abs(top_outer['start'][1] - top_inner['start'][1])
        right_outer_width = abs(right_outer['start'][0] - right_inner['start'][0])
        track_width = 0.5 * (top_outer_width + right_outer_width)
        radius = max(self.city_corner_radius, 0.5 * track_width)

        self.city_min_x = left_x
        self.city_max_x = right_x
        self.city_min_y = bottom_y
        self.city_max_y = top_y
        self.city_corner_radius = radius
        self._build_city_path()
        return True

    def _build_racing_path_from_world(self, world_file_path):
        segments = self._parse_wall_segments(world_file_path)
        if not segments:
            return False

        horizontal, _vertical = self._split_walls_by_orientation(segments)
        if len(horizontal) < 2:
            return False

        horizontal = sorted(horizontal, key=lambda seg: (seg['start'][1] + seg['end'][1]) * 0.5)
        top_wall = horizontal[-1]
        bottom_wall = horizontal[0]
        top_y = 0.5 * (top_wall['start'][1] + top_wall['end'][1])
        bottom_y = 0.5 * (bottom_wall['start'][1] + bottom_wall['end'][1])
        self.racing_track_top_y = top_y
        self.racing_track_bottom_y = bottom_y
        track_center_y = 0.5 * (top_y + bottom_y)
        track_width = abs(top_y - bottom_y)
        half_lane = 0.25 * track_width

        obstacles = [seg for seg in segments if 'obstacle' in seg['name']]
        self.racing_obstacles = []
        if obstacles:
            obstacle_entries = []
            for seg in obstacles:
                center_x = 0.5 * (seg['start'][0] + seg['end'][0])
                center_y = 0.5 * (seg['start'][1] + seg['end'][1])
                half_length = 0.5 * abs(seg['end'][0] - seg['start'][0])
                blocked_side = 'left' if center_y >= track_center_y else 'right'
                obstacle_entries.append({
                    'x': center_x,
                    'y': center_y,
                    'half_length': half_length,
                    'half_width': 0.5 * min(seg['length'], seg['width']),
                    'blocked_side': blocked_side,
                })

            obstacle_entries.sort(key=lambda item: item['x'])
            self.racing_obstacles = obstacle_entries
            if len(obstacle_entries) >= 1:
                self.racing_obstacle_1_x = obstacle_entries[0]['x']
            if len(obstacle_entries) >= 2:
                self.racing_obstacle_2_x = obstacle_entries[1]['x']

        max_lane_offset = (
            0.5 * track_width
            - 0.5 * self.racing_vehicle_width
            - self.racing_wall_margin
        )
        desired_lane_offset = min(half_lane, max(max_lane_offset, 0.0))
        if self.racing_obstacles:
            limiting_offsets = []
            top_available = top_y - self.racing_wall_margin
            bottom_available = -bottom_y - self.racing_wall_margin
            for obstacle in self.racing_obstacles:
                obstacle_offset = abs(obstacle['y'] - track_center_y)
                obstacle_half_width = obstacle['half_width']
                left_offset = 0.5 * (
                    top_available - obstacle_offset + obstacle_half_width
                )
                right_offset = 0.5 * (
                    bottom_available - obstacle_offset + obstacle_half_width
                )
                limiting_offsets.extend([left_offset, right_offset])
            desired_lane_offset = min(
                desired_lane_offset,
                max(min(limiting_offsets), 0.0),
            )

        self.racing_lane_left = track_center_y + desired_lane_offset
        self.racing_lane_right = track_center_y - desired_lane_offset
        return True

    def _build_racing_lane_plan(self):
        """Create a geometry-aware lane plan from parsed obstacles.

        The vehicle starts in the left lane. If an obstacle blocks the current lane,
        transition to the opposite lane early enough that the full vehicle body clears
        the obstacle front face before the maneuver ends.
        """
        self.racing_lane_plan = []
        current_lane = self.racing_lane_left
        half_vehicle_length = 0.5 * self.racing_vehicle_length

        for obstacle in self.racing_obstacles:
            obstacle_lane = (
                self.racing_lane_left
                if obstacle['blocked_side'] == 'left'
                else self.racing_lane_right
            )
            if abs(obstacle_lane - current_lane) > 1e-6:
                continue

            obstacle_front_x = obstacle['x'] - obstacle['half_length']
            transition_start_x = obstacle['x'] - self.racing_lane_change_buffer
            transition_end_x = (
                obstacle_front_x
                - half_vehicle_length
                - self.racing_obstacle_longitudinal_margin
            )

            if transition_end_x <= transition_start_x + 0.25:
                transition_end_x = transition_start_x + 0.25

            target_lane = (
                self.racing_lane_right
                if current_lane == self.racing_lane_left
                else self.racing_lane_left
            )
            self.racing_lane_plan.append({
                'start_x': transition_start_x,
                'end_x': transition_end_x,
                'from_lane': current_lane,
                'to_lane': target_lane,
            })
            current_lane = target_lane

    def _build_racing_path_samples(self):
        self.racing_path = []
        self.racing_s = []

        if self.racing_world_file:
            world_path = self._locate_world_file(self.racing_world_file)
            if world_path:
                self._build_racing_path_from_world(world_path)

        self._build_racing_lane_plan()

        resolution = max(self.racing_path_resolution, 0.01)
        s_value = 0.0
        while s_value <= self.racing_track_length + 1e-6:
            lane, lane_slope = self._racing_path_at(s_value)
            heading = math.atan2(lane_slope, 1.0)
            self.racing_s.append(s_value)
            self.racing_path.append({
                'x': s_value,
                'y': lane,
                'heading': heading,
            })
            s_value += resolution

        self.racing_path_length = self.racing_track_length

    def _build_city_path_samples(self):
        self.city_path_samples = []
        self.city_s_samples = []

        if self.city_world_file:
            world_path = self._locate_world_file(self.city_world_file)
            if world_path:
                self._build_city_path_from_world(world_path)

        if not self.city_path:
            return

        resolution = max(self.city_path_resolution, 0.01)
        s_value = 0.0
        while s_value <= self.city_path_length + 1e-6:
            x_pos, y_pos, heading, _, _, _ = self._city_path_pose(s_value)
            self.city_s_samples.append(s_value)
            self.city_path_samples.append({
                'x': x_pos,
                'y': y_pos,
                'heading': heading,
            })
            s_value += resolution

    def _racing_nearest_index(self):
        if not self.racing_s:
            return 0
        index = bisect.bisect_left(self.racing_s, self.current_x)
        index = clamp(index, 0, len(self.racing_s) - 1)
        return int(index)

    def _racing_closest_index(self):
        if not self.racing_path:
            return 0
        center_index = self._racing_nearest_index()
        start = max(center_index - self.racing_search_window, 0)
        end = min(center_index + self.racing_search_window, len(self.racing_path) - 1)
        best_index = center_index
        best_distance_sq = None
        for idx in range(start, end + 1):
            point = self.racing_path[idx]
            dx = self.current_x - point['x']
            dy = self.current_y - point['y']
            distance_sq = dx * dx + dy * dy
            if best_distance_sq is None or distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_index = idx
        return best_index

    def _city_nearest_index(self):
        if not self.city_path_samples:
            return 0
        start = max(self.city_last_index - self.city_search_window, 0)
        end = min(
            self.city_last_index + self.city_search_window,
            len(self.city_path_samples) - 1,
        )
        best_index = self.city_last_index
        best_distance_sq = None
        for idx in range(start, end + 1):
            point = self.city_path_samples[idx]
            dx = self.current_x - point['x']
            dy = self.current_y - point['y']
            distance_sq = dx * dx + dy * dy
            if best_distance_sq is None or distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_index = idx
        return best_index

    def _city_closest_index(self):
        return self._city_nearest_index()

    def _city_heading_is_turn(self, index):
        if not self.city_path_samples:
            return False
        current = self.city_path_samples[index]['heading']
        ahead_index = min(index + 1, len(self.city_path_samples) - 1)
        ahead = self.city_path_samples[ahead_index]['heading']
        delta = abs(self._wrap_angle_positive(ahead - current))
        if delta > math.pi:
            delta = 2.0 * math.pi - delta
        return delta > self.city_curvature_threshold

    def _persist_centerlines(self):
        os.makedirs(self.centerline_output_dir, exist_ok=True)

        if self.racing_path:
            path = os.path.join(self.centerline_output_dir, 'track2_centerline.csv')
            if self.centerline_overwrite or not os.path.exists(path):
                with open(path, 'w', encoding='ascii') as handle:
                    handle.write('s,x,y,heading\n')
                    for s_value, point in zip(self.racing_s, self.racing_path):
                        handle.write(
                            f"{s_value:.3f},{point['x']:.4f},{point['y']:.4f},{point['heading']:.6f}\n"
                        )

        if self.city_path_samples:
            path = os.path.join(self.centerline_output_dir, 'track3_centerline.csv')
            if self.centerline_overwrite or not os.path.exists(path):
                with open(path, 'w', encoding='ascii') as handle:
                    handle.write('s,x,y,heading\n')
                    for s_value, point in zip(self.city_s_samples, self.city_path_samples):
                        handle.write(
                            f"{s_value:.3f},{point['x']:.4f},{point['y']:.4f},{point['heading']:.6f}\n"
                        )

    def _city_path_pose(self, s_position):
        if not self.city_path or self.city_path_length <= 0.0:
            return (
                self.current_x,
                self.current_y,
                self.current_yaw,
                0,
                0.0,
                0.0,
            )

        s_position = s_position % self.city_path_length
        for index, segment in enumerate(self.city_path):
            segment_end = segment['s_start'] + segment['length']
            if s_position <= segment_end or index == len(self.city_path) - 1:
                segment_s = s_position - segment['s_start']
                if segment['type'] == 'line':
                    heading = segment['heading']
                    x_pos = segment['start_x'] + math.cos(heading) * segment_s
                    y_pos = segment['start_y'] + math.sin(heading) * segment_s
                else:
                    angle = segment['start_angle'] + segment['direction'] * (
                        segment_s / segment['radius']
                    )
                    x_pos = segment['center_x'] + segment['radius'] * math.cos(angle)
                    y_pos = segment['center_y'] + segment['radius'] * math.sin(angle)
                    heading = angle + segment['direction'] * (math.pi / 2.0)
                return x_pos, y_pos, heading, index, segment_s, segment['length']

        return (
            self.current_x,
            self.current_y,
            self.current_yaw,
            0,
            0.0,
            0.0,
        )

    def _city_closest_s(self, x_pos, y_pos):
        if not self.city_path:
            return 0.0

        best_s = 0.0
        best_distance_sq = None
        for segment in self.city_path:
            if segment['type'] == 'line':
                dir_x = math.cos(segment['heading'])
                dir_y = math.sin(segment['heading'])
                dx = x_pos - segment['start_x']
                dy = y_pos - segment['start_y']
                projection = dx * dir_x + dy * dir_y
                projection = clamp(projection, 0.0, segment['length'])
                closest_x = segment['start_x'] + dir_x * projection
                closest_y = segment['start_y'] + dir_y * projection
                segment_s = projection
            else:
                rel_x = x_pos - segment['center_x']
                rel_y = y_pos - segment['center_y']
                angle = math.atan2(rel_y, rel_x)
                if segment['direction'] > 0.0:
                    progress = self._wrap_angle_positive(
                        angle - segment['start_angle']
                    )
                else:
                    progress = self._wrap_angle_positive(
                        segment['start_angle'] - angle
                    )
                progress = clamp(progress, 0.0, segment['span'])
                angle_clamped = segment['start_angle'] + segment['direction'] * progress
                closest_x = segment['center_x'] + segment['radius'] * math.cos(
                    angle_clamped
                )
                closest_y = segment['center_y'] + segment['radius'] * math.sin(
                    angle_clamped
                )
                segment_s = progress * segment['radius']

            distance_sq = (
                (x_pos - closest_x) ** 2 + (y_pos - closest_y) ** 2
            )
            if best_distance_sq is None or distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_s = segment['s_start'] + segment_s

        return best_s

    def _city_speed_for_segment(self, segment_index, segment_s, segment_length):
        segment = self.city_path[segment_index]
        base_speed = (
            self.city_turn_speed
            if segment['type'] == 'arc'
            else self.city_straight_speed
        )
        blend_distance = max(self.city_speed_blend_distance, 1e-3)

        if segment_s < blend_distance:
            previous_segment = self.city_path[segment_index - 1]
            if previous_segment['type'] != segment['type']:
                previous_speed = (
                    self.city_turn_speed
                    if previous_segment['type'] == 'arc'
                    else self.city_straight_speed
                )
                progress = smoothstep(segment_s / blend_distance)
                base_speed = lerp(previous_speed, base_speed, progress)

        distance_to_end = segment_length - segment_s
        if distance_to_end < blend_distance:
            next_segment = self.city_path[(segment_index + 1) % len(self.city_path)]
            if next_segment['type'] != segment['type']:
                next_speed = (
                    self.city_turn_speed
                    if next_segment['type'] == 'arc'
                    else self.city_straight_speed
                )
                progress = smoothstep(
                    1.0 - (distance_to_end / blend_distance)
                )
                base_speed = lerp(base_speed, next_speed, progress)

        return base_speed

    def _racing_path_at(self, x_position):
        current_lane = self.racing_lane_left
        for transition in self.racing_lane_plan:
            if x_position < transition['start_x']:
                return current_lane, 0.0
            if x_position <= transition['end_x']:
                transition_span = max(
                    transition['end_x'] - transition['start_x'],
                    1e-3,
                )
                transition_progress = clamp(
                    (x_position - transition['start_x']) / transition_span,
                    0.0,
                    1.0,
                )
                blend = quintic_blend(transition_progress)
                lane = lerp(
                    transition['from_lane'],
                    transition['to_lane'],
                    blend,
                )
                lane_slope = (
                    (transition['to_lane'] - transition['from_lane'])
                    * quintic_blend_derivative(transition_progress)
                    / transition_span
                )
                return lane, lane_slope
            current_lane = transition['to_lane']

        return current_lane, 0.0

    def plan_empty_track(self):
        """Drive the empty track as a centerline align-cruise-stop state machine."""
        plausible_pose = (
            -0.5 <= self.current_x <= (self.empty_track_length + 1.0)
            and abs(self.current_y) <= 0.5
        )
        if not plausible_pose:
            self.empty_track_has_valid_pose = False
            self.empty_track_last_state = 'waiting_for_valid_pose'
            self.empty_track_last_wall_clearance = -1.0
            return (0.0, 0.0, self.empty_track_heading)

        self.empty_track_has_valid_pose = True
        front_bumper_x = (
            self.current_x
            + 0.5 * self.empty_track_vehicle_length * math.cos(self.current_yaw)
        )
        finish_x = self.empty_track_length
        if front_bumper_x >= finish_x - self.empty_track_stop_position_tolerance:
            self.empty_track_stop_latched = True

        lane_target = self.empty_track_lane
        transition_distance = self.empty_track_lane_transition_distance
        if transition_distance > 1e-6:
            # Smoothly move from centerline (0.0) to the requested lane target
            # over the first N meters. Heading remains per empty_track_heading.
            progress = clamp(self.current_x / transition_distance, 0.0, 1.0)
            desired_lane = lerp(0.0, lane_target, smoothstep(progress))
        else:
            desired_lane = lane_target

        # Optional second lane target: after empty_track_lane_second_start (meters),
        # transition from the first lane to the second lane and hold it.
        if self.current_x >= self.empty_track_lane_second_start:
            second_target = self.empty_track_lane_second
            second_transition = self.empty_track_lane_second_transition_distance

            # Desired lane at the switch start (accounts for the initial transition).
            lane_at_switch = lane_target
            if transition_distance > 1e-6:
                switch_progress = clamp(
                    self.empty_track_lane_second_start / transition_distance,
                    0.0,
                    1.0,
                )
                lane_at_switch = lerp(0.0, lane_target, smoothstep(switch_progress))

            if second_transition > 1e-6:
                progress = clamp(
                    (self.current_x - self.empty_track_lane_second_start)
                    / second_transition,
                    0.0,
                    1.0,
                )
                desired_lane = lerp(
                    lane_at_switch,
                    second_target,
                    smoothstep(progress),
                )
            else:
                desired_lane = second_target

        lateral_error = desired_lane - self.current_y
        heading_error = normalize_angle(self.empty_track_heading - self.current_yaw)
        heading_tolerance_rad = math.radians(self.empty_track_align_heading_tolerance_deg)
        wall_clearance = self._empty_track_wall_clearance()

        if self.empty_track_stop_latched:
            planner_state = 'stop'
            desired_speed = 0.0
        elif (
            abs(lateral_error) > self.empty_track_align_lateral_tolerance
            or abs(heading_error) > heading_tolerance_rad
        ):
            planner_state = 'align'
            desired_speed = self.empty_track_align_speed
        else:
            planner_state = 'cruise'
            desired_speed = self.empty_track_speed

        if wall_clearance < 0.02:
            desired_speed = 0.0
        elif wall_clearance < 0.05:
            desired_speed = min(desired_speed, self.empty_track_align_speed)

        self.empty_track_last_state = planner_state
        self.empty_track_last_wall_clearance = wall_clearance
        return (
            desired_speed,
            desired_lane,
            self.empty_track_heading,
        )

    def _empty_track_wall_clearance(self):
        """Return the current body clearance to the empty-track walls."""
        half_length = 0.5 * self.empty_track_vehicle_length
        half_width = 0.5 * self.empty_track_vehicle_width
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        corners = []
        for local_x in (-half_length, half_length):
            for local_y in (-half_width, half_width):
                world_y = self.current_y + local_x * sin_yaw + local_y * cos_yaw
                corners.append(world_y)

        top_wall_y = 0.35
        bottom_wall_y = -0.35
        max_corner_y = max(corners)
        min_corner_y = min(corners)
        return min(
            top_wall_y - max_corner_y - self.empty_track_wall_margin,
            min_corner_y - bottom_wall_y - self.empty_track_wall_margin,
        )

    def plan_racing_track(self):
        """Plan the mandated obstacle-avoidance lane changes for Track 02."""
        if self.racing_override_lane_profile:
            desired_heading = 0.0
            desired_lane = self.racing_profile_lane_1
            in_transition = False
            # Switch 1: lane_1 -> lane_2
            if self.current_x >= self.racing_profile_switch_1_start:
                span = self.racing_profile_switch_1_distance
                if span > 1e-6:
                    progress = clamp(
                        (self.current_x - self.racing_profile_switch_1_start) / span,
                        0.0,
                        1.0,
                    )
                    in_transition = progress < 1.0
                    desired_lane = lerp(
                        self.racing_profile_lane_1,
                        self.racing_profile_lane_2,
                        smoothstep(progress),
                    )
                else:
                    desired_lane = self.racing_profile_lane_2

            # Switch 2: lane_2 -> lane_3
            if self.current_x >= self.racing_profile_switch_2_start:
                span = self.racing_profile_switch_2_distance
                if span > 1e-6:
                    progress = clamp(
                        (self.current_x - self.racing_profile_switch_2_start) / span,
                        0.0,
                        1.0,
                    )
                    in_transition = in_transition or (progress < 1.0)
                    desired_lane = lerp(
                        self.racing_profile_lane_2,
                        self.racing_profile_lane_3,
                        smoothstep(progress),
                    )
                else:
                    desired_lane = self.racing_profile_lane_3

            desired_speed = (
                self.racing_lane_change_speed if in_transition else self.racing_straight_speed
            )
            if self.current_x >= self.racing_track_length:
                desired_speed = 0.0
            return desired_speed, desired_lane, desired_heading

        if not self.racing_path:
            self._build_racing_path_samples()

        closest_index = self._racing_closest_index()
        self.racing_last_index = closest_index
        lookahead_steps = max(int(self.racing_lookahead / self.racing_path_resolution), 1)
        ref_index = min(closest_index + lookahead_steps, len(self.racing_path) - 1)
        anchor_point = self.racing_path[closest_index]
        ref_point = self.racing_path[ref_index]

        desired_heading = ref_point['heading']
        desired_lane = path_normal_coordinate(
            anchor_point['x'],
            anchor_point['y'],
            desired_heading,
        )

        path_coordinate = path_normal_coordinate(
            self.current_x,
            self.current_y,
            desired_heading,
        )
        lateral_error = desired_lane - path_coordinate

        desired_speed = self.racing_straight_speed
        if abs(desired_heading) > self.racing_heading_threshold:
            desired_speed = self.racing_lane_change_speed
        if abs(lateral_error) > self.racing_lane_settle_tolerance:
            desired_speed = self.racing_lane_change_speed
        wall_clearance, obstacle_clearance = self._racing_current_clearances()
        min_clearance = min(wall_clearance, obstacle_clearance)
        if min_clearance < 0.02:
            desired_speed = 0.0
        elif min_clearance < 0.05:
            desired_speed = min(desired_speed, 0.12)
        elif min_clearance < 0.08:
            desired_speed = min(desired_speed, 0.18)
        if self.current_x >= self.racing_track_length:
            desired_speed = 0.0

        self.racing_last_wall_clearance = wall_clearance
        self.racing_last_obstacle_clearance = obstacle_clearance

        return desired_speed, desired_lane, desired_heading

    def _racing_current_clearances(self):
        """Return conservative wall/obstacle clearances for the current pose."""
        half_length = 0.5 * self.racing_vehicle_length
        half_width = 0.5 * self.racing_vehicle_width
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        corners = []
        for local_x in (-half_length, half_length):
            for local_y in (-half_width, half_width):
                world_x = self.current_x + local_x * cos_yaw - local_y * sin_yaw
                world_y = self.current_y + local_x * sin_yaw + local_y * cos_yaw
                corners.append((world_x, world_y))

        max_corner_y = max(corner[1] for corner in corners)
        min_corner_y = min(corner[1] for corner in corners)
        wall_clearance = min(
            self.racing_track_top_y - max_corner_y,
            min_corner_y - self.racing_track_bottom_y,
        )

        obstacle_clearance = float('inf')
        for obstacle in self.racing_obstacles:
            obstacle_min_x = obstacle['x'] - obstacle['half_length']
            obstacle_max_x = obstacle['x'] + obstacle['half_length']
            obstacle_min_y = obstacle['y'] - obstacle['half_width']
            obstacle_max_y = obstacle['y'] + obstacle['half_width']
            rect_min_x = min(corner[0] for corner in corners)
            rect_max_x = max(corner[0] for corner in corners)
            rect_min_y = min(corner[1] for corner in corners)
            rect_max_y = max(corner[1] for corner in corners)

            x_gap = max(
                obstacle_min_x - rect_max_x,
                rect_min_x - obstacle_max_x,
                0.0,
            )
            y_gap = max(
                obstacle_min_y - rect_max_y,
                rect_min_y - obstacle_max_y,
                0.0,
            )
            if x_gap == 0.0 and y_gap == 0.0:
                obstacle_clearance = -1.0
                break
            obstacle_clearance = min(
                obstacle_clearance,
                math.hypot(x_gap, y_gap),
            )

        if obstacle_clearance == float('inf'):
            obstacle_clearance = 999.0
        return wall_clearance, obstacle_clearance

    def plan_city_track(self):
        """Follow the provided MS4 city track in a clockwise lap."""
        if not self.city_path_samples:
            self._build_city_path_samples()
        if not self.city_path_samples:
            return self.plan_empty_track()

        closest_index = self._city_closest_index()
        self.city_last_index = closest_index
        lookahead_steps = max(int(self.city_lookahead / self.city_path_resolution), 1)
        ref_index = (closest_index + lookahead_steps) % len(self.city_path_samples)

        anchor_point = self.city_path_samples[closest_index]
        ref_point = self.city_path_samples[ref_index]
        desired_heading = ref_point['heading']
        desired_lane = path_normal_coordinate(
            anchor_point['x'],
            anchor_point['y'],
            desired_heading,
        )

        is_turn = self._city_heading_is_turn(ref_index)
        desired_speed = self.city_turn_speed if is_turn else self.city_straight_speed

        return desired_speed, desired_lane, desired_heading


def main(args=None):
    """Entry point for the MS4 planning node."""
    rclpy.init(args=args)
    node = AutonomousSystemsMS4PlanningTeam23()
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
