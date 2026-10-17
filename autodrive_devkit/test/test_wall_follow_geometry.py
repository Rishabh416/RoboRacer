"""Regression tests with ray-cast corridors and obstacle geometry."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
from autodrive_roboracer.wall_follow_node import ControllerConfig, WallFollowNode  # noqa: E402


def controller():
    node = WallFollowNode.__new__(WallFollowNode)
    node.config = ControllerConfig()
    node.prev_steering = node.prev_throttle = 0.0
    node.speed = 2.0
    node.track_width = None
    node.single_wall = False
    node.reset_controller()
    return node


def command(node, scan):
    points = node.scan_points(scan)
    left, right, heading, curvature = node.wall_measurements(points)
    error = (left - right) / 2.0
    c = node.config
    anticipation = (c.heading_gain * heading
                    + math.atan(c.curvature_gain * curvature))
    steering = node.compute_steering(error, 0.025, anticipation)
    clearance = node.forward_clearance(points)
    speed = node.compute_target_speed(
        steering, clearance, left, right, curvature)
    return steering, speed


def scan_segments(segments, pose=(0.0, 0.0, 0.0)):
    """Ray-cast line segments from a LiDAR pose, without vehicle dimensions."""
    x, y, yaw = pose
    origin = np.array([x, y])
    angles = np.linspace(-3 * math.pi / 4, 3 * math.pi / 4, 1081)
    directions = np.array([np.cos(angles + yaw), np.sin(angles + yaw)]).T
    ranges = np.full(len(angles), 10.0)
    for a, b in segments:
        a, b = np.array(a, dtype=float), np.array(b, dtype=float)
        edge, relative = b - a, a - origin
        cross = directions[:, 0] * edge[1] - directions[:, 1] * edge[0]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (relative[0] * edge[1] - relative[1] * edge[0]) / cross
            u = (relative[0] * directions[:, 1] - relative[1] * directions[:, 0]) / cross
        hit = (t >= 0) & (u >= 0) & (u <= 1)
        ranges = np.minimum(ranges, np.where(hit, t, 10.0))
    return SimpleNamespace(ranges=ranges.tolist(), angle_min=angles[0],
                           angle_increment=angles[1] - angles[0],
                           range_min=0.06, range_max=10.0)


def corridor(width=2.0, end=20.0):
    return [((-5, -width / 2), (end, -width / 2)),
            ((-5, width / 2), (end, width / 2)),
            ((end, -width / 2), (end, width / 2))]


def bend(radius, width=1.6, sign=1):
    segments = []
    for r in (radius - width / 2, radius + width / 2):
        points = [(r * math.sin(a), sign * (radius - r * math.cos(a)))
                  for a in np.linspace(-math.pi / 3, math.pi, 150)]
        segments.extend(zip(points, points[1:]))
    return segments


def test_clear_straight_is_fast_and_centered():
    steering, speed = command(controller(), scan_segments(corridor()))
    assert abs(steering) < 0.05
    assert speed > 3.5


@pytest.mark.parametrize('sign', [-1, 1])
def test_tight_turn_requires_more_steering_and_less_speed(sign):
    gentle = command(controller(), scan_segments(bend(6, sign=sign)))
    tight = command(controller(), scan_segments(bend(1.5, sign=sign)))
    assert sign * tight[0] > sign * gentle[0] > 0
    assert tight[1] < gentle[1]


def test_approaching_wall_reduces_speed_before_contact():
    speeds = [command(controller(), scan_segments(corridor(end=d)))[1]
              for d in (6, 3, 1.2)]
    assert speeds[0] > speeds[1] > speeds[2]


def test_blocked_front_and_too_narrow_gap_stop():
    assert command(controller(), scan_segments(corridor(end=0.18)))[1] == 0
    assert command(controller(), scan_segments(corridor(width=0.4)))[1] == 0


def test_single_close_beam_is_not_erased():
    scan = scan_segments(corridor())
    scan.ranges[len(scan.ranges) // 2] = 0.15
    assert command(controller(), scan)[1] == 0


def test_missing_wall_returns_stop_instead_of_inventing_a_wall():
    node = controller()
    scan = scan_segments(corridor())
    for value in (math.inf, math.nan, -math.inf, 0.0):
        scan.ranges = [value] * len(scan.ranges)
        assert node.wall_measurements(node.scan_points(scan)) is None


def test_invalid_forward_beam_stops():
    node = controller()
    scan = scan_segments(corridor())
    scan.ranges[len(scan.ranges) // 2] = math.nan
    assert node.forward_clearance(node.scan_points(scan)) == 0


def test_target_speed_decreases_monotonically_with_front_distance():
    node = controller()
    distances = (6, 4, 2, 1, 0.5, 0.21, 0.2, 0.1)
    speeds = [node.compute_target_speed(0, d, 2, 2, 0) for d in distances]
    assert all(a >= b for a, b in zip(speeds, speeds[1:]))
    assert speeds[-2:] == [0, 0]
    assert 0 < speeds[-3] < speeds[0]


def test_speed_controller_releases_drive_and_limits_acceleration():
    node = controller()
    node.speed, node.prev_throttle = 3.0, 0.8
    assert node.compute_throttle(1, 0.025) == 0
    node.speed = 0
    assert node.compute_throttle(0, 0.025) == 0
    node.prev_throttle = 0
    assert node.compute_throttle(4, 0.025) <= node.config.throttle_rise_rate * 0.025


def test_throttle_is_continuous_at_target_speed():
    node = controller()
    node.prev_throttle = 0.8
    node.speed = 2.999
    below = node.compute_throttle(3, 0.025)
    # Compare the same actuator history on either side of the target. Slew
    # limiting intentionally changes successive commands even at fixed speed.
    node.prev_throttle, node.speed_integral = 0.8, 0.0
    node.speed = 3.001
    above = node.compute_throttle(3, 0.025)
    assert abs(below - above) < 0.002


def test_steers_away_from_near_side_wall():
    for offset in (-0.5, 0.5):
        steering, _ = command(controller(), scan_segments(corridor(), pose=(0, offset, 0)))
        assert offset * steering < 0


def test_anticipation_does_not_kick_derivative():
    node = controller()
    node.compute_steering(0.1, 0.025, 0)
    node.compute_steering(0.1, 0.025, 3)
    assert node.derivative == 0


def test_steering_rate_limit_applies_in_both_directions():
    node = controller()
    for error in (4, 4, -4, -4):
        previous = node.prev_steering
        steering = node.compute_steering(error, 0.025, error)
        assert abs(steering - previous) <= node.config.steering_rate * 0.025 + 1e-12
        assert -1 <= steering <= 1


def test_first_measured_width_replaces_nominal_width_without_blending():
    node = controller()
    scan = scan_segments(corridor(width=2.4))
    node.wall_measurements(node.scan_points(scan))
    assert node.track_width == pytest.approx(2.4)


def test_reset_clears_feedback_history_but_keeps_track_width():
    node = controller()
    node.track_width = 2.4
    node.compute_steering(0.1, 0.025)
    node.speed_integral = 0.2
    node.reset_controller()
    assert node.previous_error is None
    assert node.derivative == node.speed_integral == 0
    assert node.track_width == 2.4


def test_config_rejects_unsafe_values():
    for kwargs in ({'front_corridor_half_width': 0}, {'max_speed': math.nan},
                   {'max_throttle': 2}, {'ka': -1},
                   {'crawl_distance': 0.1}, {'minimum_track_width': 0},
                   {'side_slow_distance': 0.1}, {'curvature_slowdown': -1},
                   {'min_speed': ControllerConfig().max_speed + 1}):
        with pytest.raises(ValueError):
            ControllerConfig(**kwargs).validate()


def test_approach_target_at_configured_lidar_clearance():
    node = controller()
    c = node.config
    scan = scan_segments(corridor(end=c.crawl_distance))
    distance = node.forward_clearance(node.scan_points(scan))
    assert distance == pytest.approx(c.crawl_distance)
    assert command(node, scan)[1] == pytest.approx(c.min_speed)


def test_front_approach_slows_continuously_then_stops():
    node = controller()
    c = node.config
    speeds = [node.approach_speed(d) for d in (c.crawl_distance + 2,
              c.crawl_distance + 1, c.crawl_distance + 0.1,
              c.crawl_distance, (c.crawl_distance + c.stop_buffer) / 2,
              c.stop_buffer, c.stop_buffer / 2)]
    assert all(a >= b for a, b in zip(speeds, speeds[1:]))
    assert speeds[3] == pytest.approx(c.min_speed)
    assert 0 < speeds[4] < c.min_speed
    assert speeds[5:] == [0, 0]
    assert node.approach_speed(c.crawl_distance + 1e-6) == pytest.approx(c.min_speed)


def test_curvature_reduces_speed_before_steering_moves():
    node = controller()
    speeds = [node.compute_target_speed(0, 20, 2, 2, curvature)
              for curvature in (0, 0.3, 1.2)]
    assert speeds[0] > speeds[1] > speeds[2]


@pytest.mark.parametrize('sign', [-1, 1])
def test_strong_steering_corrections_reduce_speed_on_straight_track(sign):
    node = controller()
    speeds = [node.compute_target_speed(sign * s, 20, 2, 2, 0)
              for s in (0, 0.3, 1)]
    assert speeds[0] > speeds[1] > speeds[2] > 0


def test_front_corridor_excludes_side_points_but_detects_off_axis_obstacle():
    node = controller()
    scan = scan_segments(corridor(), pose=(0, 0.75, 0))
    assert node.forward_clearance(node.scan_points(scan)) > 5
    scan = scan_segments(corridor() + [((1, 0.1), (1, 0.19))])
    assert node.forward_clearance(node.scan_points(scan)) == pytest.approx(1)


def test_side_distance_thresholds_slow_then_stop():
    node = controller()
    c = node.config
    distances = (c.side_slow_distance, (c.side_slow_distance + c.side_stop_distance) / 2,
                 c.side_stop_distance + 0.01, c.side_stop_distance)
    speeds = [node.compute_target_speed(0, 20, distance, 2, 0) for distance in distances]
    assert speeds[0] > speeds[1] > speeds[2] > speeds[3] == 0


def test_scan_coordinates_are_relative_to_lidar():
    node = controller()
    scan = scan_segments(corridor(end=2))
    x, y, _, valid, _ = node.scan_points(scan)
    center = len(scan.ranges) // 2
    assert valid[center]
    assert x[center] == pytest.approx(2)
    assert y[center] == pytest.approx(0, abs=1e-12)


def test_sensor_range_limit_is_not_fitted_as_a_wall():
    node = controller()
    scan = scan_segments(corridor())
    scan.ranges = [scan.range_max] * len(scan.ranges)
    assert node.wall_measurements(node.scan_points(scan)) is None


@pytest.mark.parametrize('sign', [-1, 1])
def test_one_visible_wall_keeps_driving_at_reduced_speed(sign):
    node = controller()
    scan = scan_segments(corridor())
    assert node.wall_measurements(node.scan_points(scan)) is not None
    angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    scan.ranges = np.where(sign * angles > math.radians(25),
                           scan.range_max, scan.ranges).tolist()
    steering, speed = command(node, scan)
    assert abs(steering) < 0.05
    assert 0 < speed <= node.config.single_wall_speed


def test_isolated_off_axis_dropout_does_not_stop():
    node = controller()
    scan = scan_segments(corridor())
    index = round((math.radians(20) - scan.angle_min) / scan.angle_increment)
    scan.ranges[index] = math.nan
    assert node.forward_clearance(node.scan_points(scan)) > 5
    assert command(node, scan)[1] > 0


def test_forward_no_return_is_clear_but_not_a_fitted_wall():
    node = controller()
    scan = scan_segments(corridor())
    angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    scan.ranges = np.where(np.abs(angles) < math.radians(10),
                           math.inf, scan.ranges).tolist()
    assert node.forward_clearance(node.scan_points(scan)) > 5
    assert node.wall_measurements(node.scan_points(scan)) is not None


def test_close_side_wall_slows_without_latching_a_stop():
    node = controller()
    scan = scan_segments(corridor(), pose=(0, 0.75, 0))
    steering, speed = command(node, scan)
    assert steering < 0
    assert speed > 0


def test_throttle_holds_drive_at_target_speed():
    node = controller()
    node.speed = 2
    for _ in range(100):
        throttle = node.compute_throttle(2, 0.025)
    assert 0.05 < throttle < node.config.max_throttle


def test_mild_overspeed_reduces_throttle_smoothly():
    node = controller()
    node.speed, node.prev_throttle = 2.1, 0.3
    previous = node.prev_throttle
    throttle = node.compute_throttle(2, 0.025)
    assert 0 < throttle < previous
    assert previous - throttle <= node.config.throttle_fall_rate * 0.025 + 1e-9


@pytest.mark.parametrize('target,speed', [(0, 1), (1, 2)])
def test_stop_or_large_overspeed_bypasses_throttle_filter(target, speed):
    node = controller()
    node.speed, node.prev_throttle = speed, 0.4
    assert node.compute_throttle(target, 0.025) == 0


def test_steering_reacts_to_bend_within_three_scans():
    node = controller()
    scan = scan_segments(bend(1.5))
    for _ in range(3):
        steering, speed = command(node, scan)
    assert steering > 0.15
    assert 0 < speed < 3
