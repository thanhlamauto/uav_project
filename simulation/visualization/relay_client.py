#!/usr/bin/env python3
"""Mac-only clone of visualization topics in an isolated ROS domain."""
import argparse
import json
import math
import socket
import threading
import time
from pathlib import Path as File
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from relay_common import TOPICS, receive


class Viewer(Node):
    def __init__(self, args):
        super().__init__('simulation_visualization_viewer')
        self.args = args
        self.pending, self.lock = {}, threading.Lock()
        self.connected, self.last_packet, self.done = False, 0, False
        self.pose, self.status, self.route_count = None, {}, 0
        self.trail = Path(); self.trail.header.frame_id = 'map'
        self.last_print = ''
        self.counts = {}
        self.pubs = []
        for topic, kind, latched, _ in TOPICS:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if latched else 5
            self.pubs.append(self.create_publisher(kind, topic, qos))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.world_pub = self.create_publisher(MarkerArray, '/visualization/forest', latched)
        self.status_pub = self.create_publisher(MarkerArray, '/visualization/status', 2)
        self.trail_pub = self.create_publisher(Path, '/visualization/actual_path', 2)
        self.world_pub.publish(self.world())
        self.worker = threading.Thread(target=self.read_stream, daemon=True)
        self.worker.start()
        self.create_timer(.02, self.forward)
        self.create_timer(.5, self.display_status)

    def read_stream(self):
        while not self.done:
            try:
                with socket.create_connection(('127.0.0.1', 7448), timeout=2) as sock:
                    sock.settimeout(2)
                    self.connected = True
                    while not self.done:
                        slot, data = receive(sock)
                        with self.lock:
                            self.pending[slot] = data
                        self.last_packet = time.monotonic()
            except (OSError, ValueError, ConnectionError) as error:
                self.connected = False
                if not self.done:
                    print('Visualization reconnect:', error, flush=True)
                    time.sleep(.5)

    def forward(self):
        with self.lock:
            batch, self.pending = self.pending, {}
        for slot, data in batch.items():
            msg = deserialize_message(data, TOPICS[slot][1])
            self.pubs[slot].publish(msg)
            topic = TOPICS[slot][0]
            self.counts[topic] = self.counts.get(topic, 0)+1
            if slot == 3:
                self.pose = msg
                p = PoseStamped(); p.header = msg.header; p.pose = msg.pose.pose
                if not self.trail.poses or math.dist(
                    [p.pose.position.x,p.pose.position.y,p.pose.position.z],
                    [self.trail.poses[-1].pose.position.x,self.trail.poses[-1].pose.position.y,self.trail.poses[-1].pose.position.z]) > .05:
                    self.trail.poses.append(p)
                    self.trail.poses = self.trail.poses[-3000:]
                self.trail.header.stamp = msg.header.stamp
            elif slot == 4:
                self.route_count = len(msg.poses)
            elif slot == 10:
                for item in msg.status:
                    self.status[item.name] = (item, time.monotonic())
        required = ('/clock','/tf','/tf_static','/localization/odometry',
                    '/camera/image_rect_color','/perception/obstacles_camera','/diagnostics')
        if all(self.counts.get(topic, 0) for topic in required):
            File(self.args.ready).touch()

    def marker(self, name, index, kind):
        m = Marker(); m.header.frame_id='map'; m.ns=name; m.id=index
        m.type=kind; m.action=Marker.ADD; m.pose.orientation.w=1
        return m

    def world(self):
        output = MarkerArray()
        for i, box in enumerate(json.loads(File(self.args.geometry).read_text())['boxes']):
            shape = box.get('shape','box')
            kind = {'cylinder':Marker.CYLINDER,'sphere':Marker.SPHERE}.get(shape,Marker.CUBE)
            m = self.marker('forest_reference',i,kind)
            m.pose.position.x,m.pose.position.y,m.pose.position.z=box['center']
            m.scale.x,m.scale.y,m.scale.z=box['size']
            if shape=='sphere':
                m.color.r,m.color.g,m.color.b,m.color.a=.15,.45,.2,.22
            elif shape=='cylinder':
                m.color.r,m.color.g,m.color.b,m.color.a=.48,.28,.12,.85
            else:
                m.color.r,m.color.g,m.color.b,m.color.a=.55,.6,.65,.45
            output.markers.append(m)
        goal=self.marker('goal',0,Marker.SPHERE)
        goal.pose.position.x,goal.pose.position.y,goal.pose.position.z=self.args.goal
        goal.scale.x=goal.scale.y=goal.scale.z=.45
        goal.color.r,goal.color.g,goal.color.b,goal.color.a=1.,.15,.8,1.
        output.markers.append(goal)
        return output

    def display_status(self):
        lines = [f'A*: {self.route_count} path points | yellow=A*, cyan=MPPI, white=flight',
                 'Orange=predicted depth points | forest=simulation reference']
        bad = not self.connected or time.monotonic()-self.last_packet>2
        if bad:
            lines.insert(0,'VISUALIZATION LINK STALE / DISCONNECTED')
        for name in ('local_navigation','autopilot_adapter','monocular_depth_cpp'):
            pair=self.status.get(name)
            if not pair:
                lines.append(name+': waiting for diagnostics');continue
            item,stamp=pair;values={v.key:v.value for v in item.values}
            stale=time.monotonic()-stamp>2.5 or float(values.get('visualization_age_ms',0))>2500
            level=item.level if isinstance(item.level,int) else int.from_bytes(item.level,'little')
            bad |= stale or level>0
            lines.append(name+': '+item.message+(' [diagnostic stale]' if stale else ''))
        m=self.marker('live_status',0,Marker.TEXT_VIEW_FACING)
        m.pose.position.x,m.pose.position.y,m.pose.position.z=10.,-3.,8.
        m.scale.z=.35;m.color.r=1. if bad else .75;m.color.g=.25 if bad else 1.
        m.color.b=.25 if bad else .8;m.color.a=1.;m.text='\n'.join(lines)
        self.status_pub.publish(MarkerArray(markers=[m]))
        self.trail_pub.publish(self.trail)
        summary = dict(wall_s=time.time(), connected=self.connected, topic_counts=self.counts,
                       route_points=self.route_count, diagnostics={name:item.message for name,(item,_) in self.status.items()},
                       diagnostic_values={name:{v.key:v.value for v in item.values} for name,(item,_) in self.status.items()},
                       rviz_subscribers={topic:[v.node_name for v in self.get_subscriptions_info_by_topic(topic)]
                                         for topic in ('/planning/global_path','/perception/obstacles_camera','/camera/image_rect_color')})
        File(self.args.ready).with_name('visualization.json').write_text(json.dumps(summary,indent=2)+'\n')
        key=' | '.join(lines[2:])
        if key!=self.last_print:
            print(f'{time.time():.3f} {key}',flush=True);self.last_print=key
            with File(self.args.ready).with_name('visualization_events.jsonl').open('a') as stream:
                stream.write(json.dumps(summary)+'\n')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--geometry',required=True)
    parser.add_argument('--ready',required=True)
    parser.add_argument('--goal',nargs=3,type=float,required=True)
    args=parser.parse_args()
    rclpy.init();node=Viewer(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.done=True
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':
    main()
