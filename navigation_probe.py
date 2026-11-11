#!/usr/bin/env python3
# 部署前被动地面探针：仅订阅PX4状态/接地及可选RC，不创建飞行发布者、不发动作。
# 主机wall用于与源微秒时间比较，monotonic用于本机接收年龄；首次观察到armed在整次探针内锁存拒绝。
"""Passive, bounded deployment observation. No flight publishers or commands."""
from __future__ import annotations
import json
import math
import os
from pathlib import Path
import subprocess
import time

from launcher import ControlError, choose_agent, process_list, uart_owners


# 维护status/land两条独立新样本链；每类至少两帧、源新鲜且单一发布者才准许地面部署。
class GroundEvidence:
    # rows保存源stamp与单调receipt；counts统计真正推进的新帧；armed_seen不能被后续上锁帧清除。
    def __init__(self):
        self.rows = {}
        self.counts = {'status': 0, 'land': 0}
        self.armed_seen = False
        self.reason = '等待新的未解锁、接地、静止消息'

    # 将PX4微秒转秒，拒绝非目标系统/解锁/非静止及过期；时间回退清该链，重复样本不累加。
    def observe(self, kind, msg, wall, mono):
        if kind == 'status' and msg.get('arming_state') == 2:
            self.armed_seen = True
            self.reason = '观察到飞机已解锁；保留原服务'
        stamp = msg.get('timestamp', 0) / 1e6
        good = -0.1 <= wall - stamp <= 2.5
        if kind == 'status':
            good = (
                good
                and msg.get('system_id') == 1
                and msg.get('component_id') == 1
                and msg.get('arming_state') == 1
            )
        else:
            good = good and msg.get('landed') is True and msg.get('at_rest') is True
        if not good:
            self.rows.pop(kind, None)
            self.counts[kind] = 0
            return
        old = self.rows.get(kind)
        if old and stamp <= old['stamp']:
            if stamp < old['stamp']:
                self.rows.pop(kind, None)
                self.counts[kind] = 0
            return
        self.rows[kind] = {'stamp': stamp, 'receipt': mono}
        self.counts[kind] += 1

    # 最终准入时重新按当前时间验证两链与图来源数量，避免先前健康样本被无限复用。
    def ready(self, wall, mono, status_publishers, land_publishers):
        if self.armed_seen:
            return False
        if status_publishers != 1 or land_publishers != 1:
            self.reason = '等待每个反馈话题恰好一个发布源'
            return False
        for kind in ('status', 'land'):
            r = self.rows.get(kind)
            if (
                not r
                or self.counts[kind] < 2
                or not 0 <= mono - r['receipt'] <= 2.5
                or not -0.1 <= wall - r['stamp'] <= 2.5
            ):
                self.reason = '等待同步时钟下连续的新鲜未解锁/接地消息'
                return False
        self.reason = '已观察到新鲜、单一来源的未解锁接地静止反馈'
        return True


# 附加RC部署证据：最低油门、三轴居中、CH5/kill关闭、有效非导航档及手动链路未丢失。
class RcGroundEvidence:
    """Additional deployment admission; observes the same RC safety fields as native."""

    # mode_slot来自生成配置，不能硬猜导航档；manual/switches/flags各有独立计数和水位。
    def __init__(self, mode_slot=6):
        self.mode_slot = mode_slot
        self.rows = {}
        self.counts = {k: 0 for k in ('manual', 'switches', 'flags')}
        self.reason = '等待新鲜 RC 安全反馈：油门最低、摇杆居中、解锁开关关闭、非导航档'

    def observe(self, kind, msg, wall, mono):
        stamp = msg.get('timestamp', 0) / 1e6
        sample = msg.get('timestamp_sample', msg.get('timestamp', 0)) / 1e6
        good = 0 < sample <= stamp and -0.1 <= wall - stamp <= 2.5 and -0.1 <= wall - sample <= 2.5
        if kind == 'manual':
            axes = [msg.get(k) for k in ('roll', 'pitch', 'yaw', 'throttle')]
            good = good and (
                msg.get('valid') is True
                and msg.get('data_source') == 1
                and msg.get('sticks_moving') is False
                and all(type(v) in (int, float) and math.isfinite(v) for v in axes)
            )
            if good:
                good = all(abs(v) <= 0.15 for v in axes[:3]) and -1.01 <= axes[3] <= -0.9
        elif kind == 'switches':
            good = good and (
                msg.get('arm_switch') == 3
                and msg.get('kill_switch') == 3
                and type(msg.get('mode_slot')) is int
                and 1 <= msg['mode_slot'] <= 6
                and msg['mode_slot'] != self.mode_slot
                and all(
                    msg.get(k) in (0, 3)
                    for k in ('return_switch', 'loiter_switch', 'offboard_switch')
                )
            )
        else:
            good = good and msg.get('manual_control_signal_lost') is False
        if not good:
            self.rows.pop(kind, None)
            self.counts[kind] = 0
            return
        old = self.rows.get(kind)
        if old and (stamp <= old['stamp'] or sample <= old['sample']):
            if stamp < old['stamp'] or sample < old['sample']:
                self.rows.pop(kind, None)
                self.counts[kind] = 0
            return
        self.rows[kind] = dict(stamp=stamp, sample=sample, receipt=mono)
        self.counts[kind] += 1

    # RC三类都需恰好一个来源且至少两帧新样本，接收和源样本年龄独立检查。
    def ready(self, wall, mono, publishers):
        return all(
            publishers.get(k) == 1
            and self.counts[k] >= 2
            and k in self.rows
            and 0 <= mono - self.rows[k]['receipt'] <= 2.5
            and -0.1 <= wall - self.rows[k]['sample'] <= 2.5
            for k in self.counts
        )


# 18秒内只观察；优先复用同用户同UART既有Agent，在线激活可要求已有Agent，禁止临时启动第二个。
def probe(
    root,
    agent_binary,
    evidence_path=None,
    *,
    require_existing_agent=False,
    require_rc_safe=False,
    rc_navigation_mode_slot=6,
):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import VehicleStatus, VehicleLandDetected

    evidence_path = evidence_path or Path.home() / 'FC01_navigation/probe.json'
    evidence_path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    evidence = GroundEvidence()
    rc = RcGroundEvidence(rc_navigation_mode_slot) if require_rc_safe else None
    agent_pid = None
    created = False
    node = None
    ok = False
    verified_at = 0.0
    try:
        match = choose_agent(process_list(), uart_owners('/dev/ttyS2'), '/dev/ttyS2', os.getuid())
        if match:
            agent_pid = match.pid
        else:
            if require_existing_agent:
                evidence.reason = '未找到可复用的既有 Agent；在线启用不会启动第二个 Agent'
                raise ControlError(evidence.reason)
            child = subprocess.Popen(
                [str(agent_binary), 'serial', '-D', '/dev/ttyS2', '-b', '115200', '-v', '0'],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            agent_pid = child.pid
            created = True
        rclpy.init()
        node = rclpy.create_node('fc01_install_ground_observer')

        # ROS回调只抽取合同字段并记录此刻wall/monotonic，不修改PX4。
        def receive(kind, msg):
            fields = (
                ('timestamp', 'system_id', 'component_id', 'arming_state')
                if kind == 'status'
                else ('timestamp', 'landed', 'at_rest')
            )
            evidence.observe(
                kind, {k: getattr(msg, k) for k in fields}, time.time(), time.monotonic()
            )

        subs = [
            node.create_subscription(
                VehicleStatus,
                '/fmu/out/vehicle_status_v1',
                lambda m: receive('status', m),
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                VehicleLandDetected,
                '/fmu/out/vehicle_land_detected',
                lambda m: receive('land', m),
                qos_profile_sensor_data,
            ),
        ]
        rc_topics = {}
        if rc is not None:
            from px4_msgs.msg import ManualControlSetpoint, ManualControlSwitches, FailsafeFlags

            rc_topics = {
                'manual': '/fmu/out/manual_control_setpoint',
                'switches': '/fmu/out/manual_control_switches',
                'flags': '/fmu/out/failsafe_flags',
            }
            rc_fields = {
                'manual': (
                    'timestamp',
                    'timestamp_sample',
                    'valid',
                    'data_source',
                    'sticks_moving',
                    'roll',
                    'pitch',
                    'yaw',
                    'throttle',
                ),
                'switches': (
                    'timestamp',
                    'timestamp_sample',
                    'arm_switch',
                    'kill_switch',
                    'mode_slot',
                    'return_switch',
                    'loiter_switch',
                    'offboard_switch',
                ),
                'flags': ('timestamp', 'manual_control_signal_lost'),
            }

            # 各lambda通过默认参数冻结kind，避免循环闭包把不同RC话题混为同一链。
            def receive_rc(kind, msg):
                rc.observe(
                    kind,
                    {k: getattr(msg, k) for k in rc_fields[kind]},
                    time.time(),
                    time.monotonic(),
                )

            for kind, cls in (
                ('manual', ManualControlSetpoint),
                ('switches', ManualControlSwitches),
                ('flags', FailsafeFlags),
            ):
                subs.append(
                    node.create_subscription(
                        cls,
                        rc_topics[kind],
                        lambda m, k=kind: receive_rc(k, m),
                        qos_profile_sensor_data,
                    )
                )
        # 等待有界；看到armed立即停止，只有全部新鲜证据满足才记录verified_at。
        deadline = time.monotonic() + 18
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if evidence.armed_seen:
                break
            ok = evidence.ready(
                time.time(),
                time.monotonic(),
                node.count_publishers('/fmu/out/vehicle_status_v1'),
                node.count_publishers('/fmu/out/vehicle_land_detected'),
            )
            if (
                ok
                and rc is not None
                and not rc.ready(
                    time.time(),
                    time.monotonic(),
                    {k: node.count_publishers(v) for k, v in rc_topics.items()},
                )
            ):
                ok = False
                evidence.reason = rc.reason
            if ok:
                verified_at = time.monotonic()
                break
        if not ok:
            raise ControlError(evidence.reason + '；未变更服务')
        print(evidence.reason, flush=True)
        return 0
    # 无论成功失败都保存本次探针身份/原因；销毁本节点，不结束既有Agent或改变服务。
    finally:
        evidence_path.write_text(
            json.dumps(
                dict(
                    ok=ok,
                    reason=evidence.reason,
                    armed_seen=evidence.armed_seen,
                    samples=evidence.counts,
                    rc_samples=rc.counts if rc else None,
                    rc_safe=ok if rc else None,
                    agent_pid=agent_pid,
                    agent_created=created,
                    agent_retained=not created,
                    updated_unix=time.time(),
                    updated_monotonic=verified_at,
                    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                ),
                ensure_ascii=False,
                indent=2,
            )
            + '\n'
        )
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
