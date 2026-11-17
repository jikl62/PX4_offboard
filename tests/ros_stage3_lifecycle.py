#!/usr/bin/env python3
"""真实 ROS 2 入口 + 本机简化机体模型；不接 Agent、串口、PX4 或真实动力。

隔离 domain、localhost 固定启用。非零目标仅被本进程模型消费。
模型结果只验证软件集成与该模型下响应，不能代替 PX4 SITL、ARM64 或实机性能。
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

SCENARIOS = ('normal', 'range_unavailable', 'feedback_stale', 'quat_reset', 'vertical_reset',
             'offboard_loss', 'offboard_flag', 'rc_takeover', 'kill', 'velocity_warning')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenario', choices=SCENARIOS, default='normal')
    parser.add_argument('--motion', choices=('plant', 'replay'), default='plant')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    if not args.node.is_file():
        parser.error('build the actual ROS 2 node first')
    # 不能仅把当前HEAD贴到一个历史ELF上：同时核对已有构建证据中的ELF和全部生产输入。
    build_evidence = root / 'docs/investigations/20261116_stage2/evidence/ROS_BUILD.json'
    build = json.loads(build_evidence.read_text())
    binary_sha = hashlib.sha256(args.node.read_bytes()).hexdigest()
    mismatches = [name for name, digest in build['production_sha256'].items()
                  if hashlib.sha256((root / name).read_bytes()).hexdigest() != digest]
    if mismatches or binary_sha != build['binary_sha256']:
        parser.error('Stage 2 build identity mismatch: ' + repr(mismatches))
    input_paths = [Path(__file__), root/'tests/fixtures/stage3_runtime_20261114.yaml',
                   build_evidence, args.node, *(root/name for name in build['production_sha256'])]
    input_hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in input_paths}
    os.environ.update(ROS_DOMAIN_ID='191', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    for key in ('ROS_DISCOVERY_SERVER', 'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'FASTDDS_DEFAULT_PROFILES_FILE', 'RMW_FASTRTPS_USE_QOS_FROM_XML', 'ROS_STATIC_PEERS'):
        os.environ.pop(key, None)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from px4_msgs.msg import (BatteryStatus, FailsafeFlags, ManualControlSetpoint,
        ManualControlSwitches, OffboardControlMode, VehicleAttitude, VehicleAttitudeSetpoint,
        VehicleCommand, VehicleCommandAck, VehicleLandDetected, VehicleLocalPosition, VehicleStatus)
    import yaml
    rclpy.init(args=[])
    source = Node('fc01_stage3_synthetic_px4')
    # 保留生产话题名：当前所有权图查询使用这些名字；remap会被图检查正确拒绝。
    # domain191 + localhost + 启动前拒绝其他节点隔离本机合成系统。
    sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
    classes = {'vehicle_status_v1': VehicleStatus, 'vehicle_land_detected': VehicleLandDetected,
        'vehicle_local_position': VehicleLocalPosition, 'vehicle_attitude': VehicleAttitude,
        'failsafe_flags': FailsafeFlags, 'battery_status': BatteryStatus,
        'manual_control_setpoint': ManualControlSetpoint, 'manual_control_switches': ManualControlSwitches,
        'vehicle_command_ack': VehicleCommandAck}
    publishers = {name: source.create_publisher(cls, '/fmu/out/' + name, sensor)
                  for name, cls in classes.items()}
    process = ipc = None
    state = {}
    records, targets, modes, commands, samples, events = [], [], [], [], [], []
    sequence = 0
    recv_buffer = b''
    start = last_step = time.monotonic()
    next_publish = next_status = start
    simulated = dict(armed=False, offboard=False, arm_switch=3, mode_slot=6, kill=3,
                     sticks=False, quat_reset=0, z_reset=0, stale=False, offboard_lost=False)
    h = speed = x = y = 0.0
    thrust = 0.0
    target_height = 1.5
    nav_at = ramp_at = hold_at = land_at = injected_at = None
    stop_at = None
    phase_trace = []
    last_phase = None
    error = None
    result = None
    cleaning = False
    ack_due = None
    ack_sent = False
    release_ack_due = None
    release_ack_sent = False
    frames = (output / 'feedback.jsonl').open('w')
    received = (output / 'received.jsonl').open('w')
    statuses = (output / 'states.jsonl').open('w')

    def event(name, **values):
        item = dict(event=name, monotonic=time.monotonic(), **values)
        events.append(item)
        return item

    def receive_target(msg):
        nonlocal thrust
        vals = list(msg.q_d) + list(msg.thrust_body)
        if not all(math.isfinite(v) for v in vals) or not (-.800001 <= msg.thrust_body[2] <= 0):
            raise RuntimeError('invalid/excessive thrust wire')
        if msg.thrust_body[0] or msg.thrust_body[1]:
            raise RuntimeError('unexpected horizontal thrust')
        if any(abs(float(a)-b) > 1.e-6 for a,b in zip(msg.q_d, [1.,0.,0.,0.])):
            raise RuntimeError('unexpected tilt/yaw in no-vision fixture')
        if not cleaning and not simulated['armed'] and any(abs(float(v)) > 1.e-6 for v in msg.thrust_body):
            raise RuntimeError('nonzero target while synthetic PX4 disarmed')
        thrust = -float(msg.thrust_body[2])
        item = dict(kind='target', received=time.monotonic(), timestamp=msg.timestamp,
                    q=[float(v) for v in msg.q_d], thrust=[float(v) for v in msg.thrust_body])
        targets.append(item)
        received.write(json.dumps(item) + '\n')

    def receive_mode(msg):
        if not msg.attitude or any((msg.position, msg.velocity, msg.acceleration, msg.body_rate,
                                   msg.thrust_and_torque, msg.direct_actuator)):
            raise RuntimeError('changed Offboard mode contract')
        item = dict(kind='heartbeat', received=time.monotonic(), timestamp=msg.timestamp)
        modes.append(item)
        received.write(json.dumps(item) + '\n')

    def receive_command(msg):
        nonlocal ack_due, release_ack_due
        # 原路线在上锁完成后可发176退出Offboard到STAB；禁止自动Arm/进入Offboard。
        disarm = msg.command==400 and msg.param1==0
        release = (msg.command==176 and msg.param1==1 and msg.param2==7
                   and not simulated['armed'] and any(c['command']==400 for c in commands))
        if not (disarm or release) or h > .15 or abs(speed) > .15:
            raise RuntimeError('unexpected/unsafe VehicleCommand: ' + str(msg))
        item = dict(kind='command', received=time.monotonic(), timestamp=msg.timestamp,
                    command=msg.command, param1=msg.param1, param2=msg.param2, height=h, speed=speed)
        commands.append(item)
        received.write(json.dumps(item) + '\n')
        # 故意分离 ACK 与状态：+150ms 满足原400因果下界，但不据 ACK 伪造已上锁。
        if disarm:
            if sum(c['command']==400 for c in commands)>1:raise RuntimeError('duplicate Disarm command')
            ack_due=time.monotonic()+.15
        else:
            if sum(c['command']==176 for c in commands)>1:raise RuntimeError('duplicate ground release')
            release_ack_due=time.monotonic()+.15
            event('post_disarm_stabilized_release_command')

    subscriptions = [source.create_subscription(VehicleAttitudeSetpoint,
        '/fmu/in/vehicle_attitude_setpoint', receive_target, reliable),
        source.create_subscription(OffboardControlMode, '/fmu/in/offboard_control_mode',
                                   receive_mode, reliable),
        source.create_subscription(VehicleCommand, '/fmu/in/vehicle_command',
                                   receive_command, reliable)]

    def publish():
        stamp = source.get_clock().now().nanoseconds // 1000
        ground = h <= .015 and abs(speed) <= .08
        range_ok = not (args.scenario == 'range_unavailable' and h < .3)
        velocity_warning = args.scenario == 'velocity_warning' and injected_at is not None
        vz = -speed + (.3 if velocity_warning else 0.)
        status = VehicleStatus(timestamp=stamp, system_id=1, component_id=1,
            arming_state=2 if simulated['armed'] else 1, nav_state=14 if simulated['offboard'] else 15,
            armed_time=10 if simulated['armed'] else 0, latest_arming_reason=2 if simulated['armed'] else 0,
            pre_flight_checks_pass=True, power_input_valid=True,
            can_set_nav_states_mask=(1 << 15), hil_state=0)
        land = VehicleLandDetected(timestamp=stamp, landed=ground, at_rest=ground,
                                  ground_contact=ground, maybe_landed=ground)
        local = VehicleLocalPosition(timestamp=stamp, timestamp_sample=stamp-2000,
            x=float(x), y=float(y), z=float(2.-h), vx=0., vy=0., vz=float(vz), z_deriv=float(-speed),
            xy_valid=True, v_xy_valid=True, z_valid=True, v_z_valid=True,
            heading_good_for_control=True, z_reset_counter=simulated['z_reset'],
            dist_bottom_valid=range_ok, dist_bottom=float(.10+h), dist_bottom_var=.0025,
            dist_bottom_sensor_bitfield=VehicleLocalPosition.DIST_BOTTOM_SENSOR_RANGE)
        attitude = VehicleAttitude(timestamp=stamp, timestamp_sample=stamp-2000,
                                  q=[1.,0.,0.,0.], quat_reset_counter=simulated['quat_reset'])
        manual = ManualControlSetpoint(timestamp=stamp, timestamp_sample=stamp-2000,
            valid=True, data_source=ManualControlSetpoint.SOURCE_RC, throttle=-1.,
            sticks_moving=simulated['sticks'], roll=.2 if simulated['sticks'] else 0.)
        switches = ManualControlSwitches(timestamp=stamp, timestamp_sample=stamp-2000,
            mode_slot=simulated['mode_slot'], arm_switch=simulated['arm_switch'],
            kill_switch=simulated['kill'], return_switch=3, loiter_switch=3, offboard_switch=3)
        payload = dict(vehicle_status_v1=status, vehicle_land_detected=land,
            vehicle_local_position=local, vehicle_attitude=attitude,
            failsafe_flags=FailsafeFlags(timestamp=stamp, offboard_control_signal_lost=simulated['offboard_lost']),
            battery_status=BatteryStatus(timestamp=stamp,id=1,connected=True,voltage_v=7.6,remaining=1.),
            manual_control_setpoint=manual, manual_control_switches=switches)
        for topic, message in payload.items():
            if simulated['stale'] and topic == 'vehicle_local_position':
                continue
            publishers[topic].publish(message)
        item = dict(received=time.monotonic(), timestamp=stamp, height=h, up_speed=speed,
                    x=x, y=y, vz=vz, z_deriv=-speed, range_valid=range_ok,
                    armed=simulated['armed'], offboard=simulated['offboard'])
        samples.append(item)
        frames.write(json.dumps(item) + '\n')

    def send(op):
        nonlocal sequence
        sequence += 1
        ipc.sendall(f'{sequence} {op}\n'.encode())
        return sequence

    def step():
        nonlocal last_step, h, speed, x, y, ipc, recv_buffer, next_publish, next_status
        nonlocal state, last_phase, ack_due, ack_sent, release_ack_due, release_ack_sent
        now = time.monotonic()
        if ack_due is not None and now>=ack_due:
            publishers['vehicle_command_ack'].publish(VehicleCommandAck(
                timestamp=source.get_clock().now().nanoseconds//1000, command=400,
                result=0, target_system=1, target_component=191, from_external=False))
            ack_due=None;ack_sent=True;event('disarm_ack_only_still_armed')
        if release_ack_due is not None and now>=release_ack_due:
            publishers['vehicle_command_ack'].publish(VehicleCommandAck(
                timestamp=source.get_clock().now().nanoseconds//1000, command=176,
                result=0, target_system=1, target_component=191, from_external=False))
            release_ack_due=None;release_ack_sent=True;event('release_ack_still_offboard')
        if process.poll() is not None:
            raise RuntimeError('native exited early ' + str(process.returncode))
        dt = min(.03, now-last_step)
        last_step = now
        if cleaning:
            h = speed = 0.
        elif args.motion == 'plant':
            # 仅验证用模型：上正单轴、hover=.5、线性推力、线性阻尼，无姿态/地效/噪声。
            # 垂向加速度m/s²=g*(thrust/hover-1)-1.4*v，积分步由真实主机时钟驱动。
            u = thrust if simulated['armed'] else 0.
            acceleration = 9.80665 * (u/.5-1.) - 1.4*speed
            speed += acceleration*dt
            h += speed*dt
            if h < 0:
                h = speed = 0.
        elif ramp_at is not None:
            # 预录型输入平滑轨迹，与控制输出无关：只验证流程与通信，不能量化控制性能。
            age = max(0., now-ramp_at-2.)
            if land_at is None:
                if age < 1.:
                    speed = .3*age
                    h = .15*age*age
                elif age < 5.:
                    speed = .3
                    h = .15+.3*(age-1.)
                elif age < 6.:
                    t = age-5.; speed = .3*(1-t); h = 1.35+.3*t-.15*t*t
                else:
                    h=1.5; speed=0.
            else:
                age = now-land_at
                if age < 1.:
                    speed=-.2*age;h=1.5-.1*age*age
                elif age < 7.5:
                    speed=-.2;h=1.4-.2*(age-1.)
                elif age < 8.5:
                    t=age-7.5;speed=-.2*(1-t);h=.1-.2*t+.1*t*t
                else:
                    h=speed=0.
        # 该高度模型没有横向力/风；xy固定只能证明水平指令未开启，不能证明实机不漂移。
        x = y = 0.
        if now >= next_publish:
            publish(); next_publish = now + .05
        # 持续消费实际DDS消息，而不是只读取本地publish结果。
        rclpy.spin_once(source, timeout_sec=.001)
        if ipc is None and socket_path.exists():
            ipc = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            ipc.connect(str(socket_path)); ipc.setblocking(False)
        if ipc is not None:
            if now >= next_status:
                send('STATUS'); next_status=now+.05
            while True:
                try:
                    data = ipc.recv(262144)
                except BlockingIOError:
                    break
                if not data:
                    raise RuntimeError('IPC disconnected')
                recv_buffer += data
                while b'\n' in recv_buffer:
                    line, recv_buffer = recv_buffer.split(b'\n',1)
                    item = json.loads(line)
                    item['_received']=time.monotonic()
                    records.append(item);statuses.write(json.dumps(item)+'\n')
                    state=item
                    phase=item.get('phase')
                    if phase and phase != last_phase:
                        phase_trace.append(dict(phase=phase, reason=item.get('reason'),
                                                received=item['_received'],height=h))
                        print(args.scenario, phase, item.get('reason'), 'h=',round(h,3), flush=True)
                        last_phase=phase
        time.sleep(.0005)

    def until(predicate, timeout, label):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            step()
            if predicate(): return
        raise RuntimeError(label+' timed out; state='+json.dumps(state))

    with tempfile.TemporaryDirectory(prefix='fc01-s3-') as tmp:
        socket_path=Path(tmp)/'control.sock'
        params=yaml.safe_load((root/'tests/fixtures/stage3_runtime_20261114.yaml').read_text())
        p=params['fc01_system']['ros__parameters']
        p.update(socket_path=str(socket_path),fc01_config_path='stage3-synthetic-runtime',
                 fc01_config_source_path='stage3-synthetic-runtime',fc01_entry_version='stage2-candidate-8767608')
        config=output/'runtime.yaml';config.write_text(yaml.safe_dump(params,sort_keys=False))
        logs=(output/'controller.log').open('w')
        try:
            # 只在本测试source独占的本机域启动；真实飞行/串口不在此进程能力范围内。
            until_graph=time.monotonic()+.4
            while time.monotonic()<until_graph:rclpy.spin_once(source,timeout_sec=.01)
            others=[name for name in source.get_node_names() if name!=source.get_name()]
            if others:raise RuntimeError('isolated domain not empty: '+repr(others))
            argv=[str(args.node.resolve()),'--ros-args','--disable-external-lib-logs','--params-file',str(config)]
            (output/'command.json').write_text(json.dumps(argv,indent=2)+'\n')
            process=subprocess.Popen(argv,stdin=subprocess.DEVNULL,stdout=logs,stderr=subprocess.STDOUT,env=os.environ.copy())
            until(lambda:state.get('phase')=='RC_READY_WAIT_ARM',12.,'ground proof/READY')
            seq=send('TAKEOFF 1.5')
            until(lambda:any(r.get('seq')==seq for r in records),2.,'console TAKEOFF refusal')
            reply=next(r for r in records if r.get('seq')==seq)
            if reply.get('accepted'):raise RuntimeError('RC route accepted console TAKEOFF')
            event('console_takeoff_refused',message=reply.get('message'))
            simulated['arm_switch']=1;event('manual_arm_switch')
            until(lambda:state.get('phase')=='RC_ARMED_WAIT_NAVIGATION',2.,'arm intent')
            simulated['armed']=True;event('simulated_manual_armed_status')
            until(lambda:state.get('reason')=='RC_ARMED_WAIT_CH6_NAVIGATION',2.,'manual arm evidence')
            simulated['mode_slot']=1;event('navigation_switch')
            # 实际源的模式反馈与开关为不同消息，分开提交。
            wait=time.monotonic()+.15
            until(lambda:time.monotonic()>=wait,.5,'navigation edge')
            simulated['offboard']=True;nav_at=time.monotonic();event('simulated_offboard_status')
            until(lambda:state.get('phase')=='THRUST_RAMP',3.,'armed dwell/Ramp')
            ramp_at=time.monotonic();event('ramp')
            until(lambda:state.get('phase')=='HEIGHT_HOLD',15.,'liftoff/Ascend/HeightHold')
            hold_at=time.monotonic();event('height_hold')
            hold_end=time.monotonic()+2.
            until(lambda:time.monotonic()>=hold_end,3.,'stable hold observation')
            if not (state.get('liftoff_confirmed') and state.get('target_height_settled')
                    and state.get('height_reference_z0') == 2):
                raise RuntimeError('missing liftoff/settling/original z0 proof')
            if args.scenario == 'range_unavailable':
                if not any(r.get('range_available') is False and r.get('phase')=='THRUST_RAMP' for r in records):
                    raise RuntimeError('ground range outage not observed by actual node')
                if not any(r.get('range_compared') for r in records):
                    raise RuntimeError('range trend never restored above ground')
                event('ground_range_unavailable_without_blocking_takeoff')
            if args.scenario in ('normal','range_unavailable'):
                seq=send('LAND');land_at=time.monotonic();event('land_request')
                until(lambda:any(r.get('seq')==seq for r in records),2.,'LAND reply')
                if not next(r for r in records if r.get('seq')==seq).get('accepted'):
                    raise RuntimeError('LAND refused')
                until(lambda:bool(commands),26.,'normal descent/touchdown/Disarm request')
                until(lambda:ack_sent and state.get('m1_400_ack_accepted'),2.,'attributable Disarm ACK')
                ack_only_start=time.monotonic()
                until(lambda:time.monotonic()>=ack_only_start+.5,1.,'ACK cannot replace new Disarmed status')
                if (not state.get('armed') or state.get('phase') in ('SAFE_DISARMED','IDLE','RC_READY_WAIT_ARM')
                    or state.get('m1_400_state_verified')):
                    raise RuntimeError('ACK alone ended armed task')
                event('ack_did_not_replace_disarmed_status')
                simulated['armed']=False;event('independent_new_disarmed_status')
                until(lambda:any(r.get('phase')=='SAFE_DISARMED' and r.get('m1_400_state_verified') for r in records),2.,'native safe Disarm terminal')
                event('native_safe_disarmed_confirmed',reason=state.get('reason'))
                until(lambda:release_ack_sent,2.,'existing post-disarm mode release')
                # 真实新状态确认完成后才模拟操作者释放导航模式；不提前绕过RequestDisarm终态。
                simulated.update(offboard=False,arm_switch=3,mode_slot=6)
                event('manual_ground_mode_release')
                until(lambda:state.get('phase')=='RC_READY_WAIT_ARM',12.,'new safe ground/reprepare')
                if state.get('armed') or any(abs(v)>0 for v in state.get('thrust_body',[])):
                    raise RuntimeError('reprepare produced unauthorized power')
                event('second_ready')
                second_ready=time.monotonic()
                until(lambda:time.monotonic()>=second_ready+.8,1.5,'second task awaits human arm')
                if state.get('phase')!='RC_READY_WAIT_ARM' or simulated['armed']:
                    raise RuntimeError('new task bypassed fresh manual arm')
            else:
                injected_at=time.monotonic()
                event('inject_'+args.scenario)
                if args.scenario=='feedback_stale':simulated['stale']=True
                elif args.scenario=='quat_reset':simulated['quat_reset']=1
                elif args.scenario=='vertical_reset':simulated['z_reset']=1
                elif args.scenario=='offboard_loss':simulated['offboard']=False
                elif args.scenario=='offboard_flag':simulated['offboard_lost']=True
                elif args.scenario=='rc_takeover':simulated['sticks']=True
                elif args.scenario=='kill':simulated['kill']=1
                elif args.scenario=='velocity_warning':pass
                if args.scenario=='velocity_warning':
                    until(lambda:state.get('vertical_feedback_rate_warning'),2.,'diagnostic velocity warning')
                    if state.get('phase')=='SAFETY':raise RuntimeError('velocity-only warning stopped control')
                    warning_start=time.monotonic()
                    before=(len(targets),len(modes))
                    until(lambda:time.monotonic()>=warning_start+2.,3.,'sustained diagnostic warning')
                    observed=[r for r in records if r.get('_received',0)>=warning_start and r.get('message')=='STATUS']
                    if (not observed or any(r.get('phase')!='HEIGHT_HOLD' or not r.get('vertical_feedback_healthy') for r in observed)
                        or min(len(targets)-before[0],len(modes)-before[1])<30):
                        raise RuntimeError('diagnostic warning failed to sustain healthy real wire')
                    event('warning_without_control_stop')
                else:
                    if args.scenario=='rc_takeover':
                        until(lambda:state.get('phase')=='DESCENT',2.,'controlled takeover request')
                        event('controlled_takeover_descent')
                        takeover_start=time.monotonic();before=(len(targets),len(modes))
                        until(lambda:time.monotonic()>=takeover_start+.6,1.2,'controlled handover still Offboard')
                        if (state.get('phase')!='DESCENT' or not state.get('m1_operator_release_requested')
                            or min(len(targets)-before[0],len(modes)-before[1])<8 or commands):
                            raise RuntimeError('controlled handover stopped before PX4 mode exit')
                        event('controlled_handover_wire_continued')
                        simulated['offboard']=False
                    until(lambda:state.get('phase')=='SAFETY',2.,'fault latch')
                    expected={'quat_reset':'ACTIVE_QUATERNION_RESET','vertical_reset':'ACTIVE_VERTICAL_ESTIMATOR_RESET',
                              'offboard_loss':'OPERATOR_TAKEOVER_LATCHED','offboard_flag':'OPERATOR_TAKEOVER_LATCHED',
                              'rc_takeover':'OPERATOR_TAKEOVER_LATCHED',
                              'kill':'OPERATOR_TAKEOVER_LATCHED'}
                    if args.scenario in expected and state.get('reason')!=expected[args.scenario]:
                        raise RuntimeError('wrong fault reason '+str(state.get('reason')))
                    if args.scenario=='feedback_stale' and not any(t in state.get('reason','') for t in ('STALE','CLOCK')):
                        raise RuntimeError('feedback stale not diagnosed')
                    cause={'offboard_loss':'PX4_LEFT_OFFBOARD','offboard_flag':'OFFBOARD_HEARTBEAT_LOST',
                           'rc_takeover':'PX4_LEFT_OFFBOARD','kill':'KILL_OR_TERMINATION'}.get(args.scenario)
                    if cause and state.get('takeover_reason')!=cause:
                        raise RuntimeError('wrong takeover cause '+str(state.get('takeover_reason')))
                    event('safety_observed',reason=state.get('reason'),takeover_reason=state.get('takeover_reason'),
                          delay_from_injection_s=time.monotonic()-injected_at)
                    safety_stamp=source.get_clock().now().nanoseconds//1000
                    first_fault=state.get('rc_first_fault_reason')
                    # 恢复健康反馈仍处armed，观察锁存和实际DDS停流；不靠重建进程伪造恢复。
                    simulated.update(stale=False,offboard=True,sticks=False,kill=3,offboard_lost=False,
                                     quat_reset=0,z_reset=0)
                    # counter恢复仅为锁存性质的对抗性注入；真实估计器counter不应回退。
                    # 允许旧戳在途包被排空；任何SAFETY观察之后新生成的wire都不允许。
                    tail_target=len(targets);tail_mode=len(modes)
                    end=time.monotonic()+.2
                    until(lambda:time.monotonic()>=end,.6,'drain queued wire')
                    before=(len(targets),len(modes))
                    end=time.monotonic()+.6
                    until(lambda:time.monotonic()>=end,1.,'fault remains silent')
                    if before!=(len(targets),len(modes)):raise RuntimeError('healthy frames resumed latched wire')
                    if state.get('phase')!='SAFETY':raise RuntimeError('healthy frames cleared fault latch')
                    if state.get('rc_first_fault_reason')!=first_fault:raise RuntimeError('fault first cause overwritten')
                    if any(m['timestamp']>safety_stamp for m in targets[tail_target:]+modes[tail_mode:]):
                        raise RuntimeError('new wire generated during safety drain/healthy tail')
                    event('latch_preserved_no_more_wire')
            stop_at=time.monotonic()
            result='PASS'
        except Exception as exc:
            error=repr(exc);result='FAIL'
            print(error,flush=True)
        finally:
            # 合成系统收尾：只模拟安全地面状态，再SIGINT正常退出。没有真实硬件动作。
            stop_at=stop_at or time.monotonic()
            cleaning=True
            simulated.update(armed=False,offboard=False,arm_switch=3,mode_slot=6,kill=3,sticks=False,stale=False,offboard_lost=False)
            h=speed=0.
            if process is not None and process.poll() is None:
                try:
                    end=time.monotonic()+.4
                    until(lambda:time.monotonic()>=end,1.,'cleanup disarmed evidence')
                    process.send_signal(signal.SIGINT)
                    end=time.monotonic()+5.
                    while process.poll() is None and time.monotonic()<end:
                        publish();rclpy.spin_once(source,timeout_sec=.005);time.sleep(.02)
                    if process.poll() is None:process.terminate();process.wait(timeout=3.)
                except Exception as exc:
                    error=(error or '')+' cleanup_error='+repr(exc)
                    result='FAIL'
                    process.kill();process.wait(timeout=3.)
            exit_code=process.returncode if process else None
            if exit_code!=0:
                result='FAIL';error=(error or '')+' native_exit='+str(exit_code)
            if ipc:ipc.close()
            logs.close();frames.close();received.close();statuses.close()
            source.destroy_node();rclpy.shutdown()
    # 指标只取正常任务结束/注入前窗口，避免清理造出的安全地面混入性能数据。
    metric_end=land_at or injected_at or stop_at or (samples[-1]['received'] if samples else start)
    climb=[s for s in samples if ramp_at and ramp_at<=s['received']<=metric_end]
    hold=[s for s in samples if hold_at and hold_at<=s['received']<=metric_end]
    # 正常任务统计到上锁指令，包含降落/恢复下降；故障场景统计到注入前。
    # 不把故障后有意停发、第二轮重新预发送的间隔假装成心跳调度抖动。
    disarm_commands=[c for c in commands if c['command']==400]
    heartbeat_end=(disarm_commands[-1]['received'] if disarm_commands else None) or injected_at or stop_at
    hb=[m for m in modes if nav_at and nav_at<=m['received']<=heartbeat_end]
    gaps=[b['received']-a['received'] for a,b in zip(hb,hb[1:])]
    wire_gaps=[(b['timestamp']-a['timestamp'])/1e6 for a,b in zip(hb,hb[1:])]
    target_window=[t for t in targets if nav_at and nav_at<=t['received']<=heartbeat_end]
    target_gaps=[b['received']-a['received'] for a,b in zip(target_window,target_window[1:])]
    def edge_gaps(messages):
        return ([messages[0]['received']-nav_at,heartbeat_end-messages[-1]['received']]
                if messages and nav_at else [math.inf])
    target_stamps=[t['timestamp'] for t in target_window]
    heartbeat_stamps=[m['timestamp'] for m in hb]
    # 回调调度可能让窗口边界只包含一半；严格比较位于窗口内部100ms的消息对。
    inner_targets=[m['timestamp'] for m in targets if nav_at and nav_at+.1<=m['received']<=heartbeat_end-.1]
    inner_modes=[m['timestamp'] for m in modes if nav_at and nav_at+.1<=m['received']<=heartbeat_end-.1]
    all_target_stamps={m['timestamp'] for m in targets}
    all_mode_stamps={m['timestamp'] for m in modes}
    stable_start=None
    settling=None
    for s in climb:
        if abs(s['height']-target_height)<=.15 and abs(s['up_speed'])<=.15:
            stable_start=stable_start or s['received']
            if s['received']-stable_start>=2. and settling is None:settling=stable_start-ramp_at
        else:stable_start=None
    metrics=dict(target_m=target_height,overshoot_m=max(0.,max((s['height'] for s in climb),default=0.)-target_height),
            settling_entry_from_ramp_s=settling,height_hold_from_ramp_s=hold_at-ramp_at if hold_at and ramp_at else None,
            hold_observation_s=metric_end-hold_at if hold_at else None,
            hold_max_abs_error_m=max((abs(s['height']-target_height) for s in hold),default=None),
            hold_rms_error_m=math.sqrt(sum((s['height']-target_height)**2 for s in hold)/len(hold)) if hold else None,
            horizontal_drift_m=max((math.hypot(s['x'],s['y']) for s in climb),default=None),
            horizontal_drift_interpretation='fixed XY input, not measured capability',
            max_heartbeat_receive_gap_s=max(gaps,default=None),max_heartbeat_wire_gap_s=max(wire_gaps,default=None),
            max_heartbeat_gap_including_window_edges_s=max(gaps+edge_gaps(hb)),
            max_target_gap_including_window_edges_s=max(target_gaps+edge_gaps(target_window)),
            heartbeat_window='Offboard confirmed through Disarm command (includes descent), or fault injection; excludes deliberate stop/reprepare')
    # 这些是本夹具的集成验收界限，非真实飞行性能规格；回放指标由输入预设。
    gates=dict(ordered_phases=all(name in [r['phase'] for r in phase_trace] for name in
               ('RC_READY_WAIT_ARM','RC_ARMED_WAIT_NAVIGATION','ARMED_GROUND','THRUST_RAMP','ASCEND','HEIGHT_HOLD')),
               converged_2s=settling is not None, hold_error_within_config=bool(hold) and metrics['hold_max_abs_error_m']<=p['aa_height_tolerance'],
               heartbeat_below_stall_budget=bool(gaps) and max(gaps)<p['aa_stall_seconds'] and max(wire_gaps)<p['aa_stall_seconds'])
    gates.update(target_and_heartbeat_window_covered=max(gaps+edge_gaps(hb)+target_gaps+edge_gaps(target_window))<p['aa_stall_seconds'],
                 monotonic_wire=all(b>a for a,b in zip(target_stamps,target_stamps[1:])) and all(b>a for a,b in zip(heartbeat_stamps,heartbeat_stamps[1:])),
                 target_heartbeat_pairs=bool(inner_targets) and bool(inner_modes) and set(inner_targets)<=all_mode_stamps and set(inner_modes)<=all_target_stamps)
    inputs_unchanged=all(hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest for name,digest in input_hashes.items())
    gates['inputs_unchanged_during_run']=inputs_unchanged
    phases=[r['phase'] for r in phase_trace]
    gates['ordered_phases']=gates['ordered_phases'] and [phases.index(n) for n in
        ('RC_READY_WAIT_ARM','RC_ARMED_WAIT_NAVIGATION','ARMED_GROUND','THRUST_RAMP','ASCEND','HEIGHT_HOLD')]==sorted(
        phases.index(n) for n in ('RC_READY_WAIT_ARM','RC_ARMED_WAIT_NAVIGATION','ARMED_GROUND','THRUST_RAMP','ASCEND','HEIGHT_HOLD'))
    if not all(gates.values()):result='FAIL';error=(error or '')+' metric/sequence gates='+repr(gates)
    report=dict(result=result,error=error,scenario=args.scenario,motion=args.motion,
        environment='x86_64 ROS 2 Humble real DDS localhost; synthetic PX4 peer; no firmware/Agent/UART/hardware',
        domain=191,visual_navigation_enabled=False,node=str(args.node.resolve()),
        node_sha256=binary_sha,build_provenance=dict(evidence=str(build_evidence.relative_to(root)),
            evidence_sha256=hashlib.sha256(build_evidence.read_bytes()).hexdigest(),
            matched_production_inputs=len(build['production_sha256']),binary_matches=True),
        harness_sha256=input_hashes[str(Path(__file__))],
        fixture_sha256=input_hashes[str(root/'tests/fixtures/stage3_runtime_20261114.yaml')],
        source_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
        native_exit=exit_code,phases=phase_trace,events=events,target_messages=len(targets),
        heartbeat_messages=len(modes),commands=commands,last_state=state,
        model=dict(acceleration='9.80665*(thrust/0.5-1)-1.4*up_speed',gravity_mps2=9.80665,
                   hover_thrust=.5,vertical_damping_per_s=1.4,horizontal_model='fixed zero, no disturbances',
                   limitations='No motor/attitude/ground-effect/delay/noise model; no physical flight claim'),
        gates=gates,metrics=metrics)
    (output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('result','error','scenario','motion','metrics')},ensure_ascii=False),flush=True)
    return 0 if result=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
