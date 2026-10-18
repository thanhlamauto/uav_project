#!/usr/bin/env python3
"""Manual flight preparation for LOCAL SITL only; never runs the planner."""
import argparse
import math
import time
from pymavlink import mavutil


def command(connection, code, *values):
    connection.mav.command_long_send(
        1, 1, code, 0, *(list(values) + [0] * (7 - len(values))))


def wait(connection, kind, predicate, timeout):
    deadline = time.monotonic() + timeout
    heartbeat_at = 0
    while time.monotonic() < deadline:
        if time.monotonic() >= heartbeat_at:
            connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            heartbeat_at = time.monotonic() + 1
        message = connection.recv_match(blocking=True, timeout=.25)
        if not message or message.get_srcSystem() != 1:
            continue
        if message.get_type() == 'STATUSTEXT':
            print('FCU:', message.text, flush=True)
        if message.get_type() == kind and predicate(message):
            return message
    raise RuntimeError(f'Timeout waiting for {kind}; no force-arm attempted')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status', 'takeoff', 'land'])
    parser.add_argument('--altitude', type=float, default=3.)
    args = parser.parse_args()
    if not math.isfinite(args.altitude) or not 2 <= args.altitude <= 5:
        parser.error('simulation altitude must be in [2,5] m')
    connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=254)
    try:
        hb = wait(connection, 'HEARTBEAT', lambda m: m.type == 2, 30)
        if args.action == 'status':
            print(f'SITL system={hb.get_srcSystem()} mode={mavutil.mode_string_v10(hb)} '
                  f'armed={bool(hb.base_mode & 128)}')
            return
        if args.action == 'land':
            connection.set_mode('LAND')
            wait(connection, 'HEARTBEAT', lambda m: not m.base_mode & 128, 60)
            print('LANDED and disarmed; now stop Docker, sender, GUI, SITL and Gazebo.')
            return
        if hb.base_mode & 128:
            raise RuntimeError('SITL already armed; land/reset before a new takeoff')
        connection.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        wait(connection, 'EKF_STATUS_REPORT', lambda m: bool(m.flags & 16), 90)
        for name in ('OA_TYPE', 'AVOID_ENABLE', 'PRX1_TYPE'):
            connection.mav.param_request_read_send(1, 1, name.encode(), -1)
            def matches(m):
                identifier = m.param_id.decode() if isinstance(m.param_id, bytes) else m.param_id
                return identifier.rstrip('\0') == name
            value = wait(connection, 'PARAM_VALUE', matches, 5)
            if value.param_value != 0:
                raise RuntimeError(f'{name} must be 0; use simulation/config/sitl.parm')
        connection.set_mode('GUIDED')
        wait(connection, 'HEARTBEAT', lambda m: m.custom_mode == 4, 10)
        # Normal arming only; FCU pre-arm checks remain active.
        for _ in range(8):
            command(connection, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            try:
                wait(connection, 'HEARTBEAT', lambda m: bool(m.base_mode & 128), 4)
                break
            except RuntimeError:
                pass
        else:
            raise RuntimeError('Arming rejected; see FCU status text')
        command(connection, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, args.altitude)
        stable_since = None
        def hovering(m):
            nonlocal stable_since
            stable = abs(-m.z - args.altitude) < .3 and math.sqrt(m.vx**2+m.vy**2+m.vz**2) < .4
            if not stable:
                stable_since = None
            elif stable_since is None:
                stable_since = time.monotonic()
            return stable_since is not None and time.monotonic() - stable_since > 2
        wait(connection, 'LOCAL_POSITION_NED', hovering, 50)
        print('HOVER READY. Start the Orin launch and sender, then publish ONE goal.')
    except BaseException:
        # A failed takeoff must not leave an unattended armed simulator.
        if args.action == 'takeoff':
            connection.set_mode('LAND')
        raise
    finally:
        connection.close()


if __name__ == '__main__':
    main()
