#!/usr/bin/env python3
"""Orin Docker subscriber -> loopback socket. Never receives control commands."""
import copy
import socket
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from diagnostic_msgs.msg import DiagnosticArray, KeyValue
from relay_common import TOPICS, pack


class Relay(Node):
    def __init__(self):
        super().__init__('simulation_visualization_sender')
        self.latest, self.last_sent, self.statuses = {}, {}, {}
        self.serial = 0
        self.subs = []
        for slot, (topic, kind, latched, hz) in enumerate(TOPICS):
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if latched else qos_profile_sensor_data
            cb = self.diagnostic if kind is DiagnosticArray else lambda m, i=slot: self.capture(i, m)
            self.subs.append(self.create_subscription(kind, topic, cb, qos))
        self.subs.append(self.create_subscription(DiagnosticArray,
            '/perception/monocular_depth/diagnostics', self.diagnostic, qos_profile_sensor_data))
        self.create_timer(.5, self.merge_diagnostics)

    def capture(self, slot, msg):
        now = time.monotonic()
        hz = TOPICS[slot][3]
        if hz and now-self.last_sent.get(slot, 0) < 1/hz:
            return
        self.last_sent[slot] = now
        self.serial += 1
        self.latest[slot] = (self.serial, pack(slot, serialize_message(msg)))

    def diagnostic(self, msg):
        for status in msg.status:
            if status.name in ('local_navigation', 'autopilot_adapter', 'monocular_depth_cpp'):
                self.statuses[status.name] = (copy.deepcopy(status), time.monotonic())

    def merge_diagnostics(self):
        msg = DiagnosticArray()
        for status, received in self.statuses.values():
            status = copy.deepcopy(status)
            status.values.append(KeyValue(key='visualization_age_ms', value=str((time.monotonic()-received)*1000)))
            msg.status.append(status)
        self.capture(10, msg)


def main():
    rclpy.init()
    node = Relay()
    listener = socket.socket()
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(('127.0.0.1', 7448))
    listener.listen(1)
    listener.settimeout(.001)
    client, sent = None, {}
    print('VISUALIZATION READY: 127.0.0.1:7448 (SSH tunnel only)', flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            if client is None:
                try:
                    client, _ = listener.accept()
                    client.settimeout(.2)
                    sent = {}
                except socket.timeout:
                    continue
            try:
                for slot, (serial, frame) in node.latest.items():
                    if sent.get(slot) != serial:
                        client.sendall(frame)
                        sent[slot] = serial
            except (OSError, TimeoutError):
                client.close()
                client = None  # Drop backlog instead of delaying ROS callbacks.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if client:
            client.close()
        listener.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
