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
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=args.domain, ROS_LOCALHOST_ONLY=args.localhost,
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from px4_msgs.msg import (BatteryStatus, FailsafeFlags, ManualControlSetpoint,
        ManualControlSwitches, VehicleAttitude, VehicleAttitudeSetpoint,
        VehicleCommand, VehicleLandDetected, VehicleLocalPosition, VehicleStatus)
    rclpy.init(args=[])
    peer = Node('fc01_prebuilt_disarmed_ground_peer')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    classes = {'vehicle_status_v1': VehicleStatus, 'vehicle_land_detected': VehicleLandDetected,
               'vehicle_local_position': VehicleLocalPosition, 'vehicle_attitude': VehicleAttitude,
               'failsafe_flags': FailsafeFlags, 'battery_status': BatteryStatus,
               'manual_control_setpoint': ManualControlSetpoint,
               'manual_control_switches': ManualControlSwitches}
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

    subs = [peer.create_subscription(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', target, 20),
            peer.create_subscription(VehicleCommand, '/fmu/in/vehicle_command', command, 20)]
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
    try:
        if args.node:
            if not args.config:
                parser.error('--node requires --config')
            process = subprocess.Popen([str(args.node), '--ros-args', '--params-file', str(args.config),
                                        '-p', 'socket_path:=' + str(ipc_path)], stdout=log, stderr=subprocess.STDOUT)
        next_status = started + 3
        while not stopped and time.monotonic() - started < args.seconds:
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
            if process and process.poll() is not None:
                raise RuntimeError('controller exited before test end: ' + str(process.returncode))
            now = time.monotonic()
            if process and now >= next_status and ipc_path.exists():
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as channel:
                    channel.settimeout(2)
                    channel.connect(str(ipc_path))
                    channel.sendall(b'1 STATUS\n')
                    data = b''
                    while b'\n' not in data:
                        data += channel.recv(262144)
                    for line in data.splitlines():
                        row = json.loads(line)
                        if row.get('kind') == 'reply':
                            states.append(row)
                next_status = now + 2
            time.sleep(.019)
    except Exception as exc:
        errors.append(repr(exc))
    finally:
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                errors.append('controller graceful shutdown timeout in isolated simulation')
                process.kill()
                process.wait()
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
