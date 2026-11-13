# Copyright 2026 Team 23
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit and simulation-reproduction tests for the Team 23 autonomy stack."""

import math
import os

from Autonomous_Systems_Project_Team_23 import (
    Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_23 as speed_mod,
    Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_23 as lateral_mod,
    Autonomous_Systems_MS_4_Planning_Team_23 as planning_mod,
    Autonomous_Systems_MS_5_Localization_Team_23 as loc_mod,
)
from nav_msgs.msg import Odometry
import numpy
import pytest
import rclpy
from rclpy.duration import Duration


@pytest.fixture(scope='module')
def ros_context():
    """Initialize and shut down rclpy once for the test module."""
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def _world_path(filename):
    """Return the path to a world SDF file in the source Worlds directory."""
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(pkg_root, 'Worlds', filename)


def test_wrap_and_normalize_angles():
    """Verify angle wrapping stays within [-pi, pi] across branch cuts."""
    assert math.isclose(loc_mod.normalize_angle(0.0), 0.0, abs_tol=1e-12)
    assert math.isclose(abs(loc_mod.normalize_angle(3.0 * math.pi)), math.pi, abs_tol=1e-9)
    assert math.isclose(abs(loc_mod.normalize_angle(-3.0 * math.pi)), math.pi, abs_tol=1e-9)
    assert math.isclose(
        lateral_mod.normalize_angle(2.0 * math.pi + 0.25),
        0.25,
        abs_tol=1e-9,
    )
    assert math.isclose(
        planning_mod.normalize_angle(-2.0 * math.pi - 0.35),
        -0.35,
        abs_tol=1e-9,
    )


def test_smoothstep_and_quintic_blend_properties():
    """Verify endpoint values, derivatives, and midpoint symmetry of blend curves."""
    for s_val in (-0.5, 0.0):
        assert planning_mod.smoothstep(s_val) == 0.0
        assert planning_mod.smoothstep_derivative(s_val) == 0.0
        assert planning_mod.quintic_blend(s_val) == 0.0
        assert planning_mod.quintic_blend_derivative(s_val) == 0.0

    for s_val in (1.0, 1.5):
        assert planning_mod.smoothstep(s_val) == 1.0
        assert planning_mod.smoothstep_derivative(s_val) == 0.0
        assert planning_mod.quintic_blend(s_val) == 1.0
        assert planning_mod.quintic_blend_derivative(s_val) == 0.0

    assert math.isclose(planning_mod.smoothstep(0.5), 0.5, abs_tol=1e-12)
    assert math.isclose(planning_mod.quintic_blend(0.5), 0.5, abs_tol=1e-12)
    assert math.isclose(planning_mod.smoothstep_derivative(0.5), 1.5, abs_tol=1e-12)
    assert math.isclose(planning_mod.quintic_blend_derivative(0.5), 1.875, abs_tol=1e-12)


def test_quaternion_and_ackermann_kinematics():
    """Verify quaternion yaw conversion and Ackermann yaw-rate mapping."""
    qz, qw = loc_mod.yaw_to_quaternion(0.6)
    msg = Odometry()
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    assert math.isclose(loc_mod.quaternion_to_yaw(msg.pose.pose.orientation), 0.6, abs_tol=1e-9)

    yaw_rate = lateral_mod.steering_to_yaw_rate(
        speed=0.5,
        steering_angle=0.25,
        wheel_base=0.255,
        max_turn_rate=2.0,
    )
    expected = 0.5 * math.tan(0.25) / 0.255
    assert math.isclose(yaw_rate, expected, abs_tol=1e-9)
    assert lateral_mod.steering_to_yaw_rate(0.5, 0.25, 0.0, 2.0) == 0.0


def test_world_sdf_parsing_and_obstacle_avoidance(ros_context, tmp_path):
    """Verify Empty, Racing, and City world SDF parsing and collision-free lane plans."""
    node = planning_mod.AutonomousSystemsMS4PlanningTeam23()
    try:
        node.centerline_output_dir = str(tmp_path)

        empty_world = _world_path('Empty_Track.world')
        assert node._build_racing_path_from_world(empty_world) is True
        assert node.racing_obstacles == []
        assert math.isclose(node.racing_track_top_y, 0.35, abs_tol=1e-6)
        assert math.isclose(node.racing_track_bottom_y, -0.35, abs_tol=1e-6)

        racing_world = _world_path('Racing_Track.world')
        assert node._build_racing_path_from_world(racing_world) is True
        assert len(node.racing_obstacles) == 2
        assert math.isclose(node.racing_obstacles[0]['x'], 4.0, abs_tol=1e-6)
        assert math.isclose(node.racing_obstacles[0]['y'], 0.1875, abs_tol=1e-6)
        assert node.racing_obstacles[0]['blocked_side'] == 'left'
        assert math.isclose(node.racing_obstacles[1]['x'], 8.0, abs_tol=1e-6)
        assert math.isclose(node.racing_obstacles[1]['y'], -0.1875, abs_tol=1e-6)
        assert node.racing_obstacles[1]['blocked_side'] == 'right'

        node._build_racing_lane_plan()
        assert len(node.racing_lane_plan) == 2

        half_veh_w = 0.5 * node.racing_vehicle_width
        half_veh_l = 0.5 * node.racing_vehicle_length
        for obs in node.racing_obstacles:
            x_min = obs['x'] - obs['half_length'] - half_veh_l
            x_max = obs['x'] + obs['half_length'] + half_veh_l
            for x_sample in numpy.linspace(x_min, x_max, 25):
                lane_y, _ = node._racing_path_at(float(x_sample))
                lateral_clearance = abs(lane_y - obs['y']) - obs['half_width'] - half_veh_w
                assert lateral_clearance > 0.0, (
                    f'Vehicle envelope overlaps obstacle at x={x_sample:.2f}'
                )

        city_world = _world_path('City_Track.world')
        assert node._build_city_path_from_world(city_world) is True
        assert node.city_path_length > 10.0
        s_mid = node._city_closest_s(0.0, 0.0)
        assert 0.0 <= s_mid <= node.city_path_length
    finally:
        node.destroy_node()


def test_speed_controller_feedforward_pid_and_slew_limit(ros_context):
    """Verify feedforward+PID speed controller step response, anti-windup, and slew limit."""
    node = speed_mod.AutonomousSystemsMS3CLRAlg1SpeedTeam23()
    try:
        node.desired_speed = 0.50
        node.actual_speed = 0.0
        node.speed_kp = 0.85
        node.speed_ki = 0.10
        node.speed_kd = 0.03
        node.max_acceleration = 0.30
        node.max_speed = 1.0
        node.last_speed_command = 0.0

        dt = 0.05
        node.last_control_time = node.get_clock().now() - Duration(nanoseconds=int(dt * 1e9))
        node.control_loop()

        # First step is capped by max_acceleration * dt = 0.30 * 0.05 = 0.015 m/s.
        assert math.isclose(node.last_speed_command, 0.015, abs_tol=1e-4)

        # Verify integral clamping on sustained positive error.
        node.speed_integral_limit = 0.20
        for _ in range(50):
            node.last_control_time = node.get_clock().now() - Duration(nanoseconds=int(dt * 1e9))
            node.control_loop()
        assert node.error_integral <= node.speed_integral_limit + 1e-9
    finally:
        node.destroy_node()


def test_lateral_stanley_softening_and_saturation(ros_context):
    """Verify Stanley lateral controller softening threshold and steering/rate limits."""
    node = lateral_mod.AutonomousSystemsMS3CLRAlg2LateralTeam23()
    try:
        node.wheel_base = 0.255
        node.max_steering_angle = 0.30
        node.max_steering_rate = 0.22
        node.lateral_kp = 0.30
        node.lateral_ki = 0.0
        node.lateral_kd = 0.10
        node.heading_kp = 0.50
        node.heading_kd = 0.15

        node.desired_lane = 0.20
        node.desired_heading = 0.0
        node.current_x = 0.0
        node.current_y = 0.0
        node.current_yaw = 0.0
        node.current_yaw_rate = 0.0
        node.current_speed = 0.0
        node.current_speed_command = 0.25
        node.applied_steering = 0.0

        dt = 0.05
        node.last_control_time = node.get_clock().now() - Duration(nanoseconds=int(dt * 1e9))
        node.control_loop()

        # At v=0, softening uses max(lateral_kd, 0.05) = 0.10 -> atan2(0.06, 0.10) = 0.5404 rad,
        # clamped to max_steering_angle (0.30), then rate-limited to 0.22 * 0.05 = 0.011 rad.
        assert math.isclose(node.applied_steering, 0.011, abs_tol=1e-4)

        # After many steps, steering saturates cleanly at max_steering_angle.
        for _ in range(100):
            node.last_control_time = node.get_clock().now() - Duration(nanoseconds=int(dt * 1e9))
            node.control_loop()
        assert math.isclose(node.applied_steering, 0.30, abs_tol=1e-6)
    finally:
        node.destroy_node()


def _simulate_kalman_trajectory(node, dt, steps, v_true, omega_fn):
    """Run Kalman predict/update across *steps* and return (pos_gain, yaw_gain)."""
    x_true, y_true, theta_true = 0.0, 0.0, 0.0
    noisy_pos_sq = []
    filt_pos_sq = []
    noisy_yaw_sq = []
    filt_yaw_sq = []

    node.state_estimate = None
    node.state_covariance = None
    for step in range(steps):
        omega_true = omega_fn(step)
        ideal = numpy.array([x_true, y_true, theta_true, v_true, omega_true])
        noisy = node.sample_noisy_measurement(ideal)

        if step == 0:
            node.initialize_filter(noisy)
            filt = node.state_estimate.copy()
        else:
            filt = node.run_predict_update(dt, noisy)

        noisy_pos_sq.append((noisy[0] - ideal[0]) ** 2 + (noisy[1] - ideal[1]) ** 2)
        filt_pos_sq.append((filt[0] - ideal[0]) ** 2 + (filt[1] - ideal[1]) ** 2)
        noisy_yaw_sq.append(loc_mod.normalize_angle(noisy[2] - ideal[2]) ** 2)
        filt_yaw_sq.append(loc_mod.normalize_angle(filt[2] - ideal[2]) ** 2)

        x_true += dt * v_true * math.cos(theta_true)
        y_true += dt * v_true * math.sin(theta_true)
        theta_true = loc_mod.normalize_angle(theta_true + dt * omega_true)

    pos_gain = 1.0 - math.sqrt(float(numpy.mean(filt_pos_sq))) / math.sqrt(
        float(numpy.mean(noisy_pos_sq))
    )
    yaw_gain = 1.0 - math.sqrt(float(numpy.mean(filt_yaw_sq))) / math.sqrt(
        float(numpy.mean(noisy_yaw_sq))
    )
    return pos_gain, yaw_gain


def test_kalman_filter_reduces_position_and_heading_rmse(ros_context):
    """Reproduce MS5 Kalman filter RMSE reduction under documented noise levels."""
    node = loc_mod.AutonomousSystemsMS5LocalizationTeam23()
    try:
        expected_r = numpy.diag([0.03**2, 0.03**2, 0.04**2, 0.05**2, 0.08**2])
        expected_q = numpy.diag([1e-8, 1e-8, 1e-8, 0.025**2, 0.05**2])
        assert numpy.allclose(node.measurement_covariance, expected_r)
        assert numpy.allclose(node.process_covariance, expected_q)

        # Straight-track run (10.34 s at 50 Hz, 0.50 m/s): >80% position & heading gain.
        straight_pos_gain, straight_yaw_gain = _simulate_kalman_trajectory(
            node, dt=0.02, steps=517, v_true=0.50, omega_fn=lambda _step: 0.0
        )
        assert straight_pos_gain > 0.80, f'Straight pos gain {straight_pos_gain:.3f}'
        assert straight_yaw_gain > 0.80, f'Straight yaw gain {straight_yaw_gain:.3f}'

        # Curved lane-change trajectory at 20 Hz: >65% position and heading gain.
        curved_pos_gain, curved_yaw_gain = _simulate_kalman_trajectory(
            node,
            dt=0.05,
            steps=200,
            v_true=0.50,
            omega_fn=lambda step: (
                0.25 * math.sin(2.0 * math.pi * step / 100.0) if step >= 60 else 0.0
            ),
        )
        assert curved_pos_gain > 0.65, f'Curved pos gain {curved_pos_gain:.3f}'
        assert curved_yaw_gain > 0.65, f'Curved yaw gain {curved_yaw_gain:.3f}'
        assert numpy.trace(node.state_covariance) < 0.01
    finally:
        node.destroy_node()
