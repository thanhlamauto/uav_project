#!/usr/bin/env python3
"""Observe one simulated run, publish one goal, persist evidence; no control bypass."""
import json
import math
import sys
import time
from collections import Counter
from pathlib import Path as File

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2


class Observer(Node):
    def __init__(self, goal):
        super().__init__('simulation_run_observer')
        self.goal = goal
        self.counts = Counter()
        self.received = {}
        self.state = None
        self.pose = None
        self.velocity = None
        self.route_points = 0
        self.diagnostics = {}
        self.positions = []
        self.subscriptions_kept = []
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        for topic, kind, handler in (
            ('/mavros/state', State, self.on_state),
            ('/localization/odometry', Odometry, self.on_pose),
            ('/perception/obstacles_camera', PointCloud2, self.on_cloud),
            ('/planning/mppi/predicted_path', Path, self.on_prediction),
            ('/control/safe_velocity_command', TwistStamped, self.on_command),
            ('/diagnostics', DiagnosticArray, self.on_diagnostics),
            ('/perception/monocular_depth/diagnostics', DiagnosticArray, self.on_diagnostics),
        ):
            self.subscriptions_kept.append(self.create_subscription(kind, topic, handler, qos_profile_sensor_data))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_kept.append(self.create_subscription(Path, '/planning/global_path', self.on_route, qos))

    def mark(self, key):
        self.counts[key] += 1
        self.received[key] = time.monotonic()

    def on_state(self, msg):
        self.mark('fcu')
        self.state = msg

    def on_pose(self, msg):
        self.mark('state')
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self.pose, self.velocity = [p.x, p.y, p.z], [v.x, v.y, v.z]
        self.positions.append({'wall_s': time.time(), 'xyz': self.pose})

    def on_cloud(self, msg):
        if msg.width * msg.height:
            self.mark('cloud')

    def on_prediction(self, msg):
        if len(msg.poses) >= 2:
            self.mark('predicted_path')

    def on_route(self, msg):
        self.mark('route')
        self.route_points = len(msg.poses)

    def on_command(self, msg):
        self.mark('commands')
        v = msg.twist.linear
        if max(abs(v.x), abs(v.y), abs(v.z), abs(msg.twist.angular.z)) > .01:
            self.mark('nonzero_commands')

    def on_diagnostics(self, msg):
        for item in msg.status:
            # ROS generated uint8 can be an int or a one-byte bytes object.
            level = item.level if isinstance(item.level, int) else int.from_bytes(item.level, 'little')
            self.diagnostics[item.name] = {'level': level, 'message': item.message}

    def ready(self):
        now = time.monotonic()
        return (self.state and self.state.connected and self.state.armed
                and self.state.mode == 'GUIDED' and self.counts['cloud'] >= 3
                and all(now - self.received.get(k, 0) < 1.5 for k in ('fcu', 'state', 'cloud')))

    def send_goal(self):
        msg = PoseStamped()
        # Do not use this observer's wall clock as a simulation timestamp.
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.goal
        msg.pose.orientation.w = 1.
        self.goal_pub.publish(msg)

    def snapshot(self, outcome):
        return {'outcome': outcome, 'goal': self.goal, 'position': self.pose,
                'route_points': self.route_points, 'counts': dict(self.counts),
                'fcu': None if self.state is None else {
                    'connected': self.state.connected, 'armed': self.state.armed, 'mode': self.state.mode},
                'diagnostics': self.diagnostics, 'positions': self.positions}


def main():
    duration, *goal = map(float, sys.argv[1:])
    if len(goal) != 3 or not all(math.isfinite(v) for v in [duration, *goal]):
        raise ValueError('Expected SECONDS X Y Z, all finite')
    rclpy.init()
    node = Observer(goal)
    outcome = 'inputs_not_ready'
    output = File('/state/summary.json')
    try:
        deadline = time.monotonic() + 45
        last_print = 0
        while time.monotonic() < deadline and not node.ready():
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() - last_print > 5:
                print('Waiting:', node.snapshot(outcome)['fcu'], dict(node.counts), flush=True)
                last_print = time.monotonic()
        if not node.ready():
            raise RuntimeError('No fresh cloud/state and connected+armed+GUIDED FCU within 45 s')
        # Wait for a replanner subscriber before the single goal publication.
        deadline = time.monotonic() + 10
        while not node.goal_pub.get_subscription_count() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not node.goal_pub.get_subscription_count():
            raise RuntimeError('No subscriber for /goal_pose')
        node.send_goal()
        print(f'GOAL SENT: {goal}. Observing for up to {duration:g} wall seconds.', flush=True)
        outcome = 'time_limit_without_confirmed_goal'
        deadline = time.monotonic() + duration
        stable_at = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            fresh = time.monotonic() - node.received.get('state', 0) < 1.5
            near = fresh and node.pose and math.dist(node.pose, goal) < .6
            stopped = node.velocity and math.sqrt(sum(v*v for v in node.velocity)) < .3
            stable_at = (stable_at or time.monotonic()) if near and stopped else None
            if stable_at and time.monotonic() - stable_at >= 2:
                outcome = 'goal_position_reached'  # Not a collision-free flight certificate.
                break
            if time.monotonic() - last_print > 5:
                print(f'xyz={node.pose} route={node.route_points} counts={dict(node.counts)}', flush=True)
                output.write_text(json.dumps(node.snapshot(outcome), indent=2) + '\n')
                last_print = time.monotonic()
        print('RESULT:', outcome, flush=True)
    finally:
        output.write_text(json.dumps(node.snapshot(outcome), indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
