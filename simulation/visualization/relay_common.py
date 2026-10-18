"""Read-only visualization protocol. Standard ROS messages, compressed CDR over SSH."""
import socket
import struct
import zlib
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, PointCloud2
from tf2_msgs.msg import TFMessage

# name, ROS type, transient-local, visualization rate cap (not algorithm rate)
TOPICS = [
    ('/clock', Clock, False, 30),
    ('/tf', TFMessage, False, 30),
    ('/tf_static', TFMessage, True, 0),
    ('/localization/odometry', Odometry, False, 15),
    ('/planning/global_path', Path, True, 0),
    ('/planning/mppi/predicted_path', Path, False, 10),
    ('/perception/obstacles_camera', PointCloud2, False, 5),
    ('/camera/image_rect_color', Image, False, 2),
    ('/control/safe_velocity_command', TwistStamped, False, 10),
    ('/mavros/setpoint_velocity/cmd_vel', TwistStamped, False, 10),
    ('/diagnostics', DiagnosticArray, False, 2),
]
HEADER = struct.Struct('!BI')
LIMIT = 4 * 1024 * 1024


def pack(slot, data):
    data = zlib.compress(data, 1)
    if len(data) > LIMIT:
        raise ValueError('visualization frame too large')
    return HEADER.pack(slot, len(data)) + data


def receive(sock):
    def exact(n):
        data = bytearray()
        while len(data) < n:
            part = sock.recv(n-len(data))
            if not part:
                raise ConnectionError('visualization stream closed')
            data.extend(part)
        return bytes(data)
    slot, size = HEADER.unpack(exact(HEADER.size))
    if slot >= len(TOPICS) or size > LIMIT:
        raise ValueError('invalid visualization frame header')
    decoder = zlib.decompressobj()
    payload = decoder.decompress(exact(size), LIMIT)
    if not decoder.eof or decoder.unconsumed_tail or decoder.unused_data:
        raise ValueError('invalid or oversized compressed frame')
    return slot, payload
