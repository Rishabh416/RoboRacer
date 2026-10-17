"""Run a bounded simulator trial and save telemetry; requires ROS and bridge.

Stop other driving nodes before use. This script never resets the simulator.
Usage: python3 test/run_controller_trial.py --seconds 120 --output trial.json
"""

import argparse
import json
import math
import time

import rclpy
from std_msgs.msg import Int32

from autodrive_roboracer.wall_follow_node import WallFollowNode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=120)
    parser.add_argument('--output', default='trial.json')
    args = parser.parse_args()
    rclpy.init()
    node = WallFollowNode()
    samples, counters = [], {'collisions': [], 'laps': []}
    subscriptions = []
    for topic, key in [('collision_count', 'collisions'), ('lap_count', 'laps')]:
        subscriptions.append(node.create_subscription(
            Int32, '/autodrive/roboracer_1/' + topic,
            lambda msg, k=key: counters[k].append(msg.data), 1))
    started = time.monotonic()
    last_sample = started
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_sample >= 0.1:
                last_sample = now
                samples.append({'time': now - started, 'speed': node.speed,
                                'throttle': node.prev_throttle,
                                'steering': node.prev_steering, 'fault': node.fault})
    finally:
        node.publish_commands(0.0, node.prev_steering)
        node.destroy_node()
        rclpy.shutdown()
        speeds = [s['speed'] for s in samples if s['speed'] is not None
                  and math.isfinite(s['speed'])]
        summary = {k: {'start': v[0], 'end': v[-1], 'delta': v[-1] - v[0]}
                   for k, v in counters.items() if v}
        summary.update({'mean_speed': sum(speeds) / max(1, len(speeds)),
                        'max_speed': max(speeds, default=0),
                        'samples': len(samples)})
        with open(args.output, 'w') as stream:
            json.dump({'summary': summary, 'samples': samples}, stream, indent=2)
        print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
