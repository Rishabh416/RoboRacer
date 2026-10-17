"""ROS wrapper regressions; automatically skipped outside a ROS environment."""

import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from autodrive_roboracer.wall_follow_node import WallFollowNode  # noqa: E402


@pytest.fixture
def controller():
    # Keep test publishers isolated from a running simulator on domain 0.
    rclpy.init(domain_id=99)
    node = WallFollowNode()
    node.commands = []
    node.publish_commands = lambda t, s: node.commands.append((t, s))
    yield node
    node.destroy_node()
    rclpy.shutdown()


def valid_scan(node):
    scan = LaserScan()
    scan.header.stamp = node.get_clock().now().to_msg()
    scan.angle_min = -3 * math.pi / 4
    scan.angle_increment = math.pi / 720
    scan.range_min = 0.06
    scan.range_max = 10.0
    scan.ranges = [10.0] * 1081
    return scan


def odometry(node, x=0.0, y=0.0):
    msg = Odometry()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.twist.twist.linear.x = x
    msg.twist.twist.linear.y = y
    return msg


def test_startup_waits_for_speed_and_scan(controller):
    node = controller
    assert node.prev_throttle == 0
    node.lidar_callback(valid_scan(node))
    assert node.commands[-1][0] == 0
    node.odom_callback(odometry(node))
    node.lidar_callback(valid_scan(node))
    assert node.commands[-1][0] == 0


def test_odom_speed_is_frame_independent(controller):
    node = controller
    node.odom_callback(odometry(node, -3.0, 4.0))
    assert node.speed == 5.0


@pytest.mark.parametrize('sensor', ['scan', 'odom'])
def test_watchdog_zeroes_drive_and_holds_steering(controller, sensor):
    node = controller
    node.last_scan_time = node.last_odom_time = time.monotonic()
    node.prev_throttle, node.prev_steering = 0.8, -0.5
    setattr(node, 'last_' + sensor + '_time', time.monotonic() - 2)
    node.check_scan_timeout()
    assert node.commands[-1] == (0.0, -0.5)
    assert node.prev_throttle == 0


def test_stale_scan_does_not_refresh_watchdog(controller):
    node = controller
    node.odom_callback(odometry(node))
    scan = valid_scan(node)
    scan.header.stamp.sec -= 1
    node.lidar_callback(scan)
    assert node.last_scan_time is None
    assert node.commands[-1][0] == 0


def test_invalid_odometry_cannot_keep_driving(controller):
    node = controller
    node.prev_throttle = 0.8
    node.odom_callback(odometry(node, math.nan))
    assert node.speed is None
    assert node.commands[-1][0] == 0


def test_invalid_scan_stops(controller):
    node = controller
    node.odom_callback(odometry(node))
    scan = valid_scan(node)
    scan.ranges = [math.nan] * 1081
    node.lidar_callback(scan)
    assert node.commands[-1][0] == 0


def test_control_cycle_prepares_scan_once(controller, monkeypatch):
    from test_wall_follow_geometry import corridor, scan_segments

    node = controller
    node.odom_callback(odometry(node))
    scan = valid_scan(node)
    scan.ranges = scan_segments(corridor()).ranges
    calls = {'scan': 0}
    prepare = node.scan_points

    def counted_prepare(msg):
        calls['scan'] += 1
        return prepare(msg)

    monkeypatch.setattr(node, 'scan_points', counted_prepare)
    node.lidar_callback(scan)
    assert calls == {'scan': 1}
    assert node.commands[-1][0] > 0


def test_shutdown_releases_drive_before_destroying_node(monkeypatch):
    from types import SimpleNamespace
    import autodrive_roboracer.wall_follow_node as module

    events = []
    node = SimpleNamespace(
        prev_steering=-0.3,
        publish_commands=lambda t, s: events.append(('command', t, s)),
        destroy_node=lambda: events.append('destroy'))

    def interrupted_spin(node):
        raise KeyboardInterrupt

    monkeypatch.setattr(module, 'WallFollowNode', lambda: node)
    monkeypatch.setattr(rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(rclpy, 'spin', interrupted_spin)
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    monkeypatch.setattr(rclpy, 'shutdown', lambda: events.append('shutdown'))
    module.main()
    assert events == [('command', 0.0, -0.3), 'destroy', 'shutdown']
