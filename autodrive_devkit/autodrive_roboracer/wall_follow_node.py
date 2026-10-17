#!/usr/bin/env python3
"""Curvature-aware LiDAR wall following with measured-speed feedback.

Distances are metres, speeds are m/s, and actuator commands are normalized.
Positive scan angles and steering point left. All clearance distances are
measured from the LiDAR. Zero throttle requests simulator idle braking.
"""

from dataclasses import dataclass, fields
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


@dataclass
class ControllerConfig:
    """ROS-overridable response gains and LiDAR distance thresholds."""

    kp: float = 0.65
    kd: float = 0.06
    ka: float = 1.5
    heading_gain: float = 1.5
    wall_lookahead: float = 1.8
    curvature_gain: float = 0.33
    curvature_slowdown: float = 3.0
    steering_slowdown: float = 2.0
    max_speed: float = 10.0
    min_speed: float = 4.0
    crawl_distance: float = 2
    stop_buffer: float = 0.20
    front_corridor_half_width: float = 0.20
    side_stop_distance: float = 0.17
    side_slow_distance: float = 0.67 
    minimum_track_width: float = 0.5
    max_throttle: float = 1
    speed_gain: float = 0.25
    speed_integral_gain: float = 0.18
    throttle_feedforward: float = 0.06
    throttle_rise_rate: float = 0.7
    throttle_fall_rate: float = 0.01
    overspeed_release: float = 0.5
    nominal_track_width: float = 1.6
    single_wall_speed: float = 1.5
    steering_rate: float = 10.0
    scan_timeout: float = 0.25
    odom_timeout: float = 0.25

    def validate(self):
        """Reject nonfinite gains and inconsistent distance or control limits."""
        for field in fields(self):
            value = getattr(self, field.name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(field.name + ' must be finite and nonnegative')
        for name in ('front_corridor_half_width', 'minimum_track_width',
                     'wall_lookahead', 'max_speed', 'scan_timeout', 'odom_timeout',
                     'steering_rate', 'throttle_rise_rate', 'throttle_fall_rate',
                     'nominal_track_width', 'single_wall_speed', 'speed_gain',
                     'overspeed_release'):
            if getattr(self, name) <= 0:
                raise ValueError(name + ' must be positive')
        if not 0 < self.max_throttle <= 1:
            raise ValueError('max_throttle must be in (0, 1]')
        if not self.stop_buffer < self.crawl_distance:
            raise ValueError('crawl_distance must exceed stop_buffer')
        if not self.side_stop_distance < self.side_slow_distance:
            raise ValueError('side_slow_distance must exceed side_stop_distance')
        if not 0 < self.min_speed <= self.max_speed:
            raise ValueError('min_speed must be in (0, max_speed]')


class WallFollowNode(Node):
    """Follow both track walls and slow before bends or front obstacles."""

    def __init__(self):
        super().__init__('wall_follow_node')
        defaults = ControllerConfig()
        values = {f.name: self.declare_parameter(
            f.name, getattr(defaults, f.name)).value for f in fields(defaults)}
        self.config = ControllerConfig(**values)
        self.config.validate()
        self.prev_steering = self.prev_throttle = 0.0
        self.speed = None
        self.last_scan_time = self.last_odom_time = None
        self.last_control_time = None
        self.fault = 'waiting for sensors'
        self.track_width = None
        self.single_wall = False
        self.reset_controller()
        prefix = '/autodrive/roboracer_1/'
        self.throttle_pub = self.create_publisher(Float32, prefix + 'throttle_command', 1)
        self.steering_pub = self.create_publisher(Float32, prefix + 'steering_command', 1)
        self.scan_sub = self.create_subscription(
            LaserScan, prefix + 'lidar', self.lidar_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, prefix + 'odom', self.odom_callback, qos_profile_sensor_data)
        self.watchdog = self.create_timer(0.05, self.check_scan_timeout)

    def reset_controller(self):
        """Clear tracking history after startup or a sensor fault."""
        self.derivative = 0.0
        self.previous_error = None
        self.speed_integral = 0.0

    def scan_points(self, scan):
        """Convert scan returns to LiDAR coordinates with validity masks."""
        ranges = np.asarray(scan.ranges, dtype=float)
        if (ranges.size < 10 or not math.isfinite(scan.angle_min)
                or not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0
                or not math.isfinite(scan.range_max) or scan.range_max <= 0
                or not math.isfinite(scan.range_min) or scan.range_min < 0
                or scan.range_min >= scan.range_max):
            return None
        angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
        finite_hit = (np.isfinite(ranges) & (ranges >= scan.range_min)
                      & (ranges > 0) & (ranges <= scan.range_max))
        # +inf is a no-return reading, not a measured wall. NaN, -inf and
        # zero remain unknown. Use the sensor range only to bound free space.
        valid = finite_hit | np.isposinf(ranges)
        # Unknown returns stay unknown; never use them to invent wall geometry.
        safe = np.where(valid, np.minimum(ranges, scan.range_max), 0.0)
        return (safe * np.cos(angles),
                safe * np.sin(angles), angles, valid,
                finite_hit & (ranges < scan.range_max * 0.995))

    def fit_wall(self, points, sign):
        """Fit one side wall: sign=+1 for left, -1 for right.

        Reject outliers, using a line for short spans and a quadratic otherwise.
        Return distance, heading and curvature y''/(1+y'^2)^1.5.
        """
        x, y, angles, _, hits = points
        mask = (hits & (sign * angles > math.radians(25))
                & (sign * angles < math.radians(135))
                & (sign * y > 0.05) & (x >= -0.15)
                & (x <= self.config.wall_lookahead))
        xx, yy = x[mask], y[mask]
        if len(xx) < 8 or np.ptp(xx) < 0.15:
            return None
        degree = 2 if np.ptp(xx) >= 0.4 else 1
        for _ in range(3):
            coefficients = np.polynomial.polynomial.polyfit(xx, yy, degree)
            residual = np.abs(yy - np.polynomial.polynomial.polyval(xx, coefficients))
            keep = residual <= max(0.04, 2.5 * float(np.median(residual)))
            if keep.all() or keep.sum() < 8 or np.ptp(xx[keep]) < 0.25:
                break
            xx, yy = xx[keep], yy[keep]
        intercept, slope = coefficients[:2]
        quadratic = coefficients[2] if degree == 2 else 0.0
        if not np.isfinite(coefficients).all() or sign * intercept <= 0:
            return None
        heading = math.atan(slope)
        curvature = 2 * quadratic / (1 + slope * slope) ** 1.5
        return sign * intercept, heading, curvature

    def wall_measurements(self, points):
        """Combine visible walls into distances, heading and signed curvature."""
        if points is None:
            return None
        fits = [self.fit_wall(points, sign) for sign in (1, -1)]
        self.single_wall = any(fit is None for fit in fits)
        if all(fit is None for fit in fits):
            return None
        if self.single_wall:
            # Use a measured wall and the last observed corridor width. Never
            # hold stale steering when neither side is observable.
            width = (self.config.nominal_track_width if self.track_width is None
                     else self.track_width)
            index = 0 if fits[0] is not None else 1
            distance, heading, curvature = fits[index]
            other = max(0.05, width - distance)
            return ((distance, other, heading, curvature) if index == 0
                    else (other, distance, heading, curvature))
        width = fits[0][0] + fits[1][0]
        if width > self.config.minimum_track_width:
            previous = width if self.track_width is None else self.track_width
            self.track_width = previous + 0.1 * (width - previous)
        return (fits[0][0], fits[1][0],
                (fits[0][1] + fits[1][1]) / 2,
                (fits[0][2] + fits[1][2]) / 2)

    def forward_clearance(self, points):
        """Nearest forward point in the detection corridor, measured from LiDAR."""
        if points is None:
            return 0.0
        x, y, angles, valid, _ = points
        # A bad beam off the driving axis should not shut down the car. Keep
        # the central safety window strict and reject broad forward dropouts.
        forward = np.abs(angles) <= math.radians(30)
        central = np.abs(angles) <= math.radians(3)
        if (not forward.any() or not central.any() or not valid[central].all()
                or np.mean(valid[forward]) < 0.97):
            return 0.0
        corridor = (valid & (np.cos(angles) > 0)
                    & (np.abs(y) <= self.config.front_corridor_half_width))
        if not corridor.any():
            return 0.0
        return max(0.0, float(np.min(x[corridor])))

    def compute_steering(self, error, dt, anticipation=0.0):
        """Combine centering PD and geometric anticipation, then rate-limit."""
        c = self.config
        dt = max(0.001, min(dt, 0.1))
        raw_derivative = (0 if self.previous_error is None
                          else (error - self.previous_error) / dt)
        self.derivative += dt / (0.08 + dt) * (raw_derivative - self.derivative)
        self.previous_error = error
        output = c.kp * error + c.kd * self.derivative + c.ka * anticipation
        desired = max(-1.0, min(1.0, output))
        change = max(-c.steering_rate * dt,
                     min(c.steering_rate * dt, desired - self.prev_steering))
        self.prev_steering = max(-1.0, min(1.0, self.prev_steering + change))
        return self.prev_steering

    def approach_speed(self, distance):
        """Approach envelope reaches min_speed at crawl_distance, then falls to zero."""
        c = self.config
        if distance <= c.stop_buffer:
            return 0.0
        if distance <= c.crawl_distance:
            return c.min_speed * (distance - c.stop_buffer) / (
                c.crawl_distance - c.stop_buffer)
        fraction = min(1.0, (distance - c.crawl_distance) / 4.0)
        return c.min_speed + (c.max_speed - c.min_speed) * fraction ** 2

    def compute_target_speed(self, steering, front_distance, left, right, curvature):
        """Choose the lowest empirical bend, visibility and clearance speed limit."""
        c = self.config
        side = min(left, right)
        if (side <= c.side_stop_distance or left + right <= c.minimum_track_width
                or front_distance <= c.stop_buffer):
            return 0.0
        bend_speed = c.max_speed / (1 + c.curvature_slowdown * abs(curvature))
        steering_speed = c.max_speed / (1 + c.steering_slowdown * abs(steering))
        side_fraction = (side - c.side_stop_distance) / (
            c.side_slow_distance - c.side_stop_distance)
        side_speed = max(0.2, c.max_speed * min(1.0, side_fraction))
        visibility_speed = (c.single_wall_speed if self.single_wall
                            else c.max_speed)
        return max(0.0, min(c.max_speed, bend_speed, steering_speed, side_speed,
                            visibility_speed, self.approach_speed(front_distance)))

    def compute_throttle(self, target_speed, dt):
        """Feedforward + PI hold speed; slew normal commands in both directions.

        Large overspeed, zero targets and faults bypass smoothing
        to request idle braking immediately. No positive minimum throttle is
        forced when the car needs to slow down.
        """
        c = self.config
        dt = max(0.001, min(dt, 0.1))
        if (self.speed is None or target_speed <= 0
                or self.speed - target_speed > c.overspeed_release):
            self.prev_throttle = 0.0
            self.speed_integral = 0.0
            return 0.0
        error = target_speed - self.speed
        old_integral = self.speed_integral
        # Bound low-speed integral authority so crawling cannot accumulate a
        # large motor command. Anti-windup also handles the actuator slew limit.
        candidate = max(0.0, min(min(c.max_throttle, target_speed * 0.3),
                               old_integral + c.speed_integral_gain * error * dt))
        raw = c.throttle_feedforward * target_speed + c.speed_gain * error + candidate
        desired = max(0.0, min(c.max_throttle, raw))
        command = max(self.prev_throttle - c.throttle_fall_rate * dt,
                      min(desired, self.prev_throttle + c.throttle_rise_rate * dt))
        if error <= 0 or raw <= command + 1e-9:
            self.speed_integral = candidate
        self.prev_throttle = max(0.0, min(c.max_throttle, command))
        return self.prev_throttle

    def fresh_stamp(self, msg, timeout):
        """Reject queued old or future sensor messages using the ROS clock."""
        stamp = msg.header.stamp
        age = self.get_clock().now().nanoseconds * 1e-9 - (stamp.sec + stamp.nanosec * 1e-9)
        return -0.05 <= age <= timeout

    def stop(self, reason):
        """Release throttle while holding the last steering on sensor failure."""
        previous_fault = self.fault
        self.fault = reason
        if previous_fault != reason:
            self.get_logger().warn('Controller stopped: ' + reason)
        self.prev_throttle = 0.0
        self.last_control_time = None
        self.reset_controller()
        self.publish_commands(0.0, self.prev_steering)

    def odom_callback(self, msg):
        """Use planar speed magnitude, independent of odometry frame."""
        velocity = msg.twist.twist.linear
        if (not math.isfinite(velocity.x) or not math.isfinite(velocity.y)
                or not self.fresh_stamp(msg, self.config.odom_timeout)):
            self.speed = None
            self.last_odom_time = None
            self.stop('invalid or stale odometry')
            return
        self.speed = math.hypot(velocity.x, velocity.y)
        self.last_odom_time = time.monotonic()

    def check_scan_timeout(self):
        """Stop even when a sensor ceases to invoke callbacks."""
        now = time.monotonic()
        if (self.last_scan_time is None or self.last_odom_time is None
                or now - self.last_scan_time > self.config.scan_timeout
                or now - self.last_odom_time > self.config.odom_timeout):
            self.stop('waiting for fresh scan and odometry')

    def lidar_callback(self, scan):
        """Estimate wall geometry and publish bounded steering and throttle."""
        now = time.monotonic()
        c = self.config
        if not self.fresh_stamp(scan, c.scan_timeout):
            self.stop('stale scan')
            return
        self.last_scan_time = now
        if (self.speed is None or self.last_odom_time is None
                or now - self.last_odom_time > c.odom_timeout):
            self.stop('waiting for odometry')
            return
        points = self.scan_points(scan)
        measurements = self.wall_measurements(points)
        if measurements is None:
            self.stop('insufficient wall measurements')
            return
        left, right, heading, curvature = measurements
        dt = (0.025 if self.last_control_time is None
              else min(0.1, max(0.001, now - self.last_control_time)))
        self.last_control_time = now
        anticipation = (c.heading_gain * heading
                        + math.atan(c.curvature_gain * curvature))
        steering = self.compute_steering((left - right) / 2, dt, anticipation)
        clearance = self.forward_clearance(points)
        target = self.compute_target_speed(steering, clearance, left, right, curvature)
        if target <= 0 and self.fault != 'insufficient obstacle clearance':
            self.get_logger().warn('Controller stopped: insufficient obstacle clearance')
        self.fault = '' if target > 0 else 'insufficient obstacle clearance'
        self.publish_commands(self.compute_throttle(target, dt), steering)

    def publish_commands(self, throttle, steering):
        """Publish simulator normalized motor and steering commands."""
        self.throttle_pub.publish(Float32(data=float(throttle)))
        self.steering_pub.publish(Float32(data=float(steering)))


def main(args=None):
    """Run the controller and release drive on clean shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = WallFollowNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_commands(0.0, node.prev_steering)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
