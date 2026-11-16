#!/usr/bin/env python3
"""隔离软件环境的未解锁地面 PX4 消息源；绝不模拟解锁或起飞。

供预编译发行测试使用，不进入运行包，不连接Agent/UART。--node可在软件环境
启动指定真实控制器；未给出时只为独立虚拟机中的服务提供合成反馈。
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=25)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--node', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--domain', default='192')
    parser.add_argument('--localhost', choices=('0', '1'), default='1')
    args = parser.parse_args()
    # Native IPC requires an owned private directory; set it at creation rather
    # than depending on the shell umask in a separate test environment.
    args.output.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=args.domain, ROS_LOCALHOST_ONLY=args.localhost,
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from px4_msgs.msg import (BatteryStatus, FailsafeFlags, ManualControlSetpoint,
        ManualControlSwitches, VehicleAttitude, VehicleAttitudeSetpoint,
        VehicleCommand, VehicleCommandAck, VehicleLandDetected, VehicleLocalPosition, VehicleStatus,
        OffboardControlMode)
    rclpy.init(args=[])
    peer = Node('fc01_prebuilt_disarmed_ground_peer')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    classes = {'vehicle_status_v1': VehicleStatus, 'vehicle_land_detected': VehicleLandDetected,
               'vehicle_local_position': VehicleLocalPosition, 'vehicle_attitude': VehicleAttitude,
               'failsafe_flags': FailsafeFlags, 'battery_status': BatteryStatus,
               'manual_control_setpoint': ManualControlSetpoint,
               'manual_control_switches': ManualControlSwitches,
               # A real PX4 peer owns one ACK publisher even while no command is
               # pending. Advertise it without fabricating any accepted command.
               'vehicle_command_ack': VehicleCommandAck}
    pubs = {name: peer.create_publisher(cls, '/fmu/out/' + name, qos) for name, cls in classes.items()}
    counts = {'feedback_batches': 0, 'targets': 0, 'commands': 0}
    errors = []

    def target(msg):
        counts['targets'] += 1
        if any(float(v) != 0 for v in msg.thrust_body):
            errors.append('nonzero thrust during permanently disarmed input')

    def command(msg):
        counts['commands'] += 1
        errors.append('unexpected VehicleCommand while no task was authorized: ' + str(msg.command))

    subs = [peer.create_subscription(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', target, qos),
            peer.create_subscription(VehicleCommand, '/fmu/in/vehicle_command', command, qos),
            peer.create_subscription(OffboardControlMode, '/fmu/in/offboard_control_mode',
                                     lambda _: None, qos)]
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    process = None
    states = []
    started = time.monotonic()
    ipc_path = args.output / 'control.sock'
    log = (args.output / 'controller.log').open('w')
    channel = None
    ipc_buffer = b''

    def publish_ground():
        # Feedback continues during IPC observation and graceful shutdown; pausing
        # the synthetic sources would test a transport outage instead of exit.
        stamp = peer.get_clock().now().nanoseconds // 1000
        msgs = dict(vehicle_status_v1=VehicleStatus(timestamp=stamp, system_id=1, component_id=1,
            arming_state=1, nav_state=15, pre_flight_checks_pass=True, power_input_valid=True,
            can_set_nav_states_mask=(1 << 15)),
            vehicle_land_detected=VehicleLandDetected(timestamp=stamp, landed=True, at_rest=True,
                ground_contact=True, maybe_landed=True),
            vehicle_local_position=VehicleLocalPosition(timestamp=stamp, timestamp_sample=stamp-2000,
                x=0., y=0., z=2., vx=0., vy=0., vz=0., z_deriv=0., xy_valid=True,
                v_xy_valid=True, z_valid=True, v_z_valid=True, heading_good_for_control=True,
                dist_bottom_valid=True, dist_bottom=.1, dist_bottom_var=.0025,
                dist_bottom_sensor_bitfield=VehicleLocalPosition.DIST_BOTTOM_SENSOR_RANGE),
            vehicle_attitude=VehicleAttitude(timestamp=stamp, timestamp_sample=stamp-2000, q=[1.,0.,0.,0.]),
            failsafe_flags=FailsafeFlags(timestamp=stamp),
            battery_status=BatteryStatus(timestamp=stamp,id=1,connected=True,voltage_v=7.6,remaining=1.),
            manual_control_setpoint=ManualControlSetpoint(timestamp=stamp,timestamp_sample=stamp-2000,
                valid=True,data_source=ManualControlSetpoint.SOURCE_RC,throttle=-1.),
            manual_control_switches=ManualControlSwitches(timestamp=stamp,timestamp_sample=stamp-2000,
                mode_slot=6, arm_switch=3,kill_switch=3,return_switch=3,loiter_switch=3,offboard_switch=3))
        for name, msg in msgs.items():
            pubs[name].publish(msg)
        counts['feedback_batches'] += 1
        rclpy.spin_once(peer, timeout_sec=.001)

    def observe_ipc():
        nonlocal channel, ipc_buffer
        if not process:
            return
        if channel is None and ipc_path.exists():
            channel = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            channel.settimeout(.1)
            channel.connect(str(ipc_path))
            channel.sendall(b'1 STATUS\n')
            channel.setblocking(False)
        if channel is None:
            return
        while True:
            try:
                block = channel.recv(262144)
            except BlockingIOError:
                break
            if not block:
                channel.close()
                channel = None
                break
            ipc_buffer += block
            if len(ipc_buffer) > 1048576:
                raise RuntimeError('isolated controller IPC buffer exceeded limit')
            while b'\n' in ipc_buffer:
                line, ipc_buffer = ipc_buffer.split(b'\n', 1)
                row = json.loads(line)
                if row.get('kind') in ('reply', 'state') and 'phase' in row:
                    # Keep periodic evidence bounded while retaining transitions.
                    if (not states or row.get('phase') != states[-1].get('phase')
                            or row.get('diagnostic_monotonic_s', 0)
                               - states[-1].get('diagnostic_monotonic_s', 0) >= 1):
                        states.append(row)
    try:
        if args.node:
            if not args.config:
                parser.error('--node requires --config')
            process = subprocess.Popen([str(args.node), '--ros-args', '--params-file', str(args.config),
                                        '-p', 'socket_path:=' + str(ipc_path)], stdout=log, stderr=subprocess.STDOUT)
        while not stopped and time.monotonic() - started < args.seconds:
            publish_ground()
            if process and process.poll() is not None:
                raise RuntimeError('controller exited before test end: ' + str(process.returncode))
            observe_ipc()
            time.sleep(.019)
    except Exception as exc:
        errors.append(repr(exc))
    finally:
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            deadline = time.monotonic() + 30
            while process.poll() is None and time.monotonic() < deadline:
                publish_ground()
                observe_ipc()
                time.sleep(.019)
            if process.poll() is None:
                errors.append('controller graceful shutdown timeout in isolated simulation')
                process.kill()
                process.wait()
        if channel is not None:
            channel.close()
        log.close()
        peer.destroy_node()
        rclpy.shutdown()
    if process:
        if process.returncode != 0:
            errors.append('controller exit=' + str(process.returncode))
        if not any(s.get('armed') is False and s.get('landed') is True and s.get('local_fresh') is True for s in states):
            errors.append('no fresh disarmed ground state observed')
    result = dict(result='FAIL' if errors else 'PASS', errors=errors, counts=counts,
                  elapsed=time.monotonic()-started, states=states,
                  environment='synthetic PX4 ROS messages, permanently disarmed, no UART/Agent/hardware',
                  controller_exit=process.returncode if process else None)
    (args.output / 'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: result[k] for k in ('result','errors','counts','controller_exit')}))
    return bool(errors)


if __name__ == '__main__':
    raise SystemExit(main())
