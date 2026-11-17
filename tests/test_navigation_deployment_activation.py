# 在线启用/回滚的顺序与失败处置测试：所有 sudo/systemctl 都由临时 fixture 接管，不改真实服务。
# 把构建身份、当前地面证据、旧进程退出、新进程验证作为独立步骤，不能用单个 active 状态替代。
import contextlib
import hashlib
import importlib
import itertools
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch

import navigation_restore as m
from navigation_probe import RcGroundEvidence


# 在临时 HOME 和 units 中模拟原/新服务、运行参数及唯一既有 Agent，记录每次外部动作。
class ActivationFixture:
    def __init__(self, root, *, standby=False):
        self.home = Path(root)
        self.entry = self.home / 'new'
        self.entry.mkdir()
        self.sdk = self.home / 'sdk'
        self.units = self.home / 'units'
        self.units.mkdir()
        self.folder = self.home / 'FC01_navigation'
        self.folder.mkdir(mode=0o700)
        self.binary = self.entry / 'install/lib/jikl_fc01_system/fc01_system_node'
        self.binary.parent.mkdir(parents=True)
        self.binary.write_bytes(b'new-controller')
        self.agent = types.SimpleNamespace(pid=55)
        self.original = m.render_unit('orangepi', self.home, self.home / 'old', self.sdk)
        (self.units / m.SERVICE).write_text(self.original)
        self.runtime = b'old runtime parameters\n'
        (self.folder / 'runtime.yaml').write_bytes(self.runtime)
        self.calls = []
        self.fail_install = False
        self.fail_probe = False
        self.stuck = False
        self.info = {
            m.SERVICE: dict(
                MainPID='10',
                ActiveState='active',
                UnitFileState='enabled',
                DropInPaths='',
                FragmentPath=str(self.units / m.SERVICE),
            ),
            m.OLD: dict(
                MainPID='0',
                ActiveState='inactive',
                UnitFileState='disabled',
                DropInPaths='',
                FragmentPath='',
            ),
        }
        if standby:
            (self.units / m.OLD).write_text(m.OLD_MARKER + '\n[Service]\nExecStart=/old/standby\n')
            self.info[m.OLD].update(
                MainPID='11',
                ActiveState='active',
                UnitFileState='enabled',
                FragmentPath=str(self.units / m.OLD),
            )
        self.evidence = types.SimpleNamespace(verify_build=self.verify_build)

    # 模拟已通过构建证据校验；专门用例可替换为失败以证明它发生在 sudo 之前。
    def verify_build(self, entry, sdk):
        self.calls.append(['EVIDENCE'])
        return {'ok': True}

    # 根据模拟服务状态返回子控制器描述，用于验证活动实例与托管父进程关系。
    def processes(self):
        return [
            types.SimpleNamespace(
                pid=42 if row['MainPID'] == '20' else 12,
                uid=os.getuid(),
                argv=('/bin/fc01_system_node',),
            )
            for row in self.info.values()
            if row['ActiveState'] == 'active'
        ]

    # 仅修改临时文件和内存服务状态；可注入安装失败或旧服务卡住，绝不调用真实 systemctl。
    def run(self, argv, **kwargs):
        self.calls.append(argv)
        if '/usr/bin/install' in argv:
            if self.fail_install:
                self.fail_install = False
                raise RuntimeError('simulated install failure')
            Path(argv[-1]).write_bytes(Path(argv[-2]).read_bytes())
        if '/usr/bin/rm' in argv:
            Path(argv[-1]).unlink()
        if '/usr/bin/systemctl' in argv:
            if 'stop' in argv and not self.stuck:
                self.info[argv[-1]].update(MainPID='0', ActiveState='inactive')
            if 'disable' in argv:
                self.info[argv[-1]]['UnitFileState'] = 'disabled'
            if 'enable' in argv:
                self.info[argv[-1]]['UnitFileState'] = (
                    'enabled-runtime' if '--runtime' in argv else 'enabled'
                )
            if 'start' in argv:
                self.info[m.SERVICE].update(MainPID='20', ActiveState='active')
                (self.folder / 'runtime.yaml').write_bytes(b'new runtime\n')
        return types.SimpleNamespace(stdout='WorkingDirectory=' + str(self.home / 'old') + '\n')

    # 写当前 boot_id/单调时间及未解锁 RC 安全证据，模拟被动 probe；失败分支保留原原因。
    def boot(self, entry, home, project, action, *args):
        self.calls.append(['BOOT', action, *map(str, args)])
        if self.fail_probe:
            (self.folder / 'probe.json').write_text(
                json.dumps({'reason': '等待每个反馈话题恰好一个发布源'})
            )
            raise RuntimeError('probe failed')
        (self.folder / 'probe.json').write_text(
            json.dumps(
                dict(
                    ok=True,
                    armed_seen=False,
                    agent_pid=self.agent.pid,
                    agent_created=False,
                    rc_safe=True,
                    updated_monotonic=time.monotonic(),
                    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                )
            )
        )

    # 集中替换所有硬件、进程树、服务及构建外部依赖，退出上下文后自动恢复原函数。
    def context(self, *, verifier=None):
        stack = contextlib.ExitStack()
        stack.enter_context(patch.dict(sys.modules, {'navigation_build_evidence': self.evidence}))
        stack.enter_context(patch.object(m, 'check_controller', return_value=self.binary))
        stack.enter_context(
            patch.object(m, 'existing_agent', return_value=(self.binary, self.agent))
        )
        stack.enter_context(patch.object(m, 'retained_agent', return_value=self.agent))
        stack.enter_context(patch.object(m, '_descendant', return_value=True))
        stack.enter_context(patch.object(m, '_controllers', side_effect=self.processes))
        stack.enter_context(patch.object(m, 'process_list', side_effect=self.processes))
        stack.enter_context(
            patch.object(m, 'unit_info', side_effect=lambda name, run: dict(self.info[name]))
        )
        stack.enter_context(patch.object(m, 'boot', side_effect=self.boot))
        stack.enter_context(
            patch.object(
                m,
                'verify_running',
                side_effect=verifier or (lambda *a, **k: {'controller_pid': 42}),
            )
        )
        return stack

    # 所有场景仍调用真实 activate 状态机，verify_timeout=0 只使 fixture 不等待。
    def activate(self, **kwargs):
        return m.activate(
            self.entry,
            self.home,
            self.sdk,
            units=self.units,
            run=self.run,
            verify_timeout=0,
            **kwargs,
        )


# 检查授权→新地面证据→旧进程退出→安装→启动的因果顺序，以及每个阶段的回滚边界。
class DeploymentActivationTests(unittest.TestCase):
    # 认证等待后必须重新 probe；备份原服务/参数/开机状态，且 probe 要求已有 Agent 和 RC 安全。
    def test_auth_probe_stop_exit_install_start_order_and_backup(self):
        with tempfile.TemporaryDirectory() as temp:
            f = ActivationFixture(temp)
            with f.context():
                backup = f.activate()
            calls = f.calls
            auth = calls.index(['sudo', '-v'])
            probe = next(i for i, a in enumerate(calls) if a[:2] == ['BOOT', 'probe'])
            stop = next(i for i, a in enumerate(calls) if 'stop' in a)
            install = next(i for i, a in enumerate(calls) if '/usr/bin/install' in a)
            start = next(i for i, a in enumerate(calls) if 'start' in a)
            self.assertLess(auth, probe)
            self.assertLess(probe, stop)
            self.assertLess(stop, install)
            self.assertLess(install, start)
            self.assertIn('--no-block', calls[stop])
            self.assertEqual((backup / m.SERVICE).read_text(), f.original)
            self.assertEqual((backup / 'runtime.yaml').read_bytes(), f.runtime)
            manifest = json.loads((backup / 'manifest.json').read_text())
            self.assertEqual(manifest['original_working_directory'], str(f.home / 'old'))
            self.assertEqual(manifest['original_unit_file_state'], 'enabled')
            self.assertFalse(any('enable' in a or 'kill' in a for a in calls))
            self.assertTrue(
                all(
                    '--require-existing-agent' in a and '--require-rc-safe' in a
                    for a in calls
                    if a[:2] == ['BOOT', 'probe']
                )
            )
            self.assertEqual(
                json.loads((f.entry / 'ACTIVATION_STATE.json').read_text())['phase'], 'active'
            )

    # 源/构建身份不可信时不允许进入任何管理员动作。
    def test_invalid_build_evidence_refuses_before_sudo_probe_or_stop(self):
        with tempfile.TemporaryDirectory() as temp:
            f = ActivationFixture(temp)

            def invalid(*a):
                raise m.ControlError('stale source digest')

            f.evidence.verify_build = invalid
            with f.context(), self.assertRaisesRegex(m.ControlError, 'stale'):
                f.activate()
            self.assertFalse(f.calls)
            self.assertEqual((f.units / m.SERVICE).read_text(), f.original)

    # 飞控不满足在线观测条件时保留旧服务并写 failed_unchanged，不能先停旧实例。
    def test_unpowered_hardware_reports_online_condition_without_mutation(self):
        with tempfile.TemporaryDirectory() as temp:
            f = ActivationFixture(temp)
            f.fail_probe = True
            with f.context(), self.assertRaisesRegex(m.ControlError, '硬件尚未满足在线启用条件'):
                f.activate()
            self.assertFalse(
                any('stop' in a or '/usr/bin/install' in a or 'start' in a for a in f.calls)
            )
            self.assertEqual((f.units / m.SERVICE).read_text(), f.original)
            self.assertEqual(
                json.loads((f.entry / 'ACTIVATION_STATE.json').read_text())['phase'],
                'failed_unchanged',
            )

    # 旧控制器未退出只能报告等待，不得 kill、改配置或启动第二个。
    def test_old_controller_exit_timeout_never_switches_or_kills(self):
        with tempfile.TemporaryDirectory() as temp:
            f = ActivationFixture(temp)
            f.stuck = True
            clock = itertools.count(1.0, 100.0)
            with (
                f.context(),
                patch.object(m.time, 'monotonic', side_effect=lambda: next(clock)),
                patch('navigation_install.ground_permit_fresh', return_value=True),
                self.assertRaisesRegex(m.ControlError, '尚未退出'),
            ):
                f.activate()
            self.assertFalse(
                any('/usr/bin/install' in a or 'start' in a or 'kill' in a for a in f.calls)
            )
            self.assertEqual((f.units / m.SERVICE).read_text(), f.original)

    # 安装步骤失败后还原服务字节与 runtime，新实例从未启动。
    def test_configuration_failure_restores_original_and_never_starts(self):
        with tempfile.TemporaryDirectory() as temp:
            f = ActivationFixture(temp)
            f.fail_install = True
            with f.context(), self.assertRaisesRegex(m.ControlError, 'simulated install failure'):
                f.activate()
            self.assertEqual((f.units / m.SERVICE).read_text(), f.original)
            self.assertEqual((f.folder / 'runtime.yaml').read_bytes(), f.runtime)
            self.assertFalse(any('start' in a for a in f.calls))

    # 新实例启动后验证失败时先安全停止它，再还原旧配置；不能自动重新启动旧实例。
    def test_verification_failure_stops_attempted_new_then_restores_runtime(self):
        with tempfile.TemporaryDirectory() as temp:
            f = ActivationFixture(temp)

            def fail(*a, **k):
                raise m.ControlError('diagnostic policy mismatch')

            with (
                f.context(verifier=fail),
                self.assertRaisesRegex(m.ControlError, 'policy mismatch'),
            ):
                f.activate()
            self.assertEqual((f.units / m.SERVICE).read_text(), f.original)
            self.assertEqual((f.folder / 'runtime.yaml').read_bytes(), f.runtime)
            self.assertEqual(sum('start' in a for a in f.calls), 1)
            self.assertEqual(sum('stop' in a for a in f.calls), 2)
            self.assertEqual(f.info[m.SERVICE]['ActiveState'], 'inactive')
            self.assertEqual(
                json.loads((f.entry / 'ACTIVATION_STATE.json').read_text())['phase'],
                'failed_restored',
            )

    # 升级可关闭旧待机自启，回滚须恢复其字节/开机状态，但不自动启动两套控制器。
    def test_managed_old_standby_boot_and_bytes_restored_without_restart(self):
        with tempfile.TemporaryDirectory() as temp:
            f = ActivationFixture(temp, standby=True)
            old_bytes = (f.units / m.OLD).read_bytes()
            with f.context():
                backup = f.activate()
            self.assertEqual(f.info[m.OLD]['UnitFileState'], 'disabled')
            self.assertEqual((f.units / m.OLD).read_bytes(), old_bytes)
            with f.context():
                m.rollback(
                    f.entry, f.home, f.sdk, backup, units=f.units, run=f.run, stop_current=True
                )
            self.assertEqual(f.info[m.OLD]['UnitFileState'], 'enabled')
            self.assertEqual((f.units / m.SERVICE).read_text(), f.original)
            self.assertEqual(f.info[m.SERVICE]['ActiveState'], 'inactive')
            self.assertEqual(f.info[m.OLD]['ActiveState'], 'inactive')
            self.assertEqual(sum('start' in a for a in f.calls), 1)

    # 同一候选已正确运行时重复启用应复用原备份，不再 probe/stop/start。
    def test_repeated_correct_activation_does_not_restart(self):
        with tempfile.TemporaryDirectory() as temp:
            f = ActivationFixture(temp)
            with f.context():
                first = f.activate()
            prior = len(f.calls)
            with f.context():
                second = f.activate()
            self.assertEqual(first, second)
            self.assertFalse(
                any(
                    'start' in a or 'stop' in a or a[:2] == ['BOOT', 'probe']
                    for a in f.calls[prior:]
                )
            )

    # 新实例安全恢复无法确认时保留其进程及配置，避免盲目覆盖仍持有控制权的实例。
    def test_failure_with_unsafe_new_controller_retains_process_and_configuration(self):
        with tempfile.TemporaryDirectory() as temp:
            f = ActivationFixture(temp)

            def fail(*a, **k):
                f.fail_probe = True
                raise m.ControlError('runtime verification failed')

            with f.context(verifier=fail), self.assertRaisesRegex(m.ControlError, '安全恢复未完成'):
                f.activate()
            self.assertEqual(sum('stop' in a for a in f.calls), 1)  # only the old controller
            self.assertEqual(f.info[m.SERVICE]['ActiveState'], 'active')
            self.assertNotEqual((f.units / m.SERVICE).read_text(), f.original)
            self.assertEqual(
                json.loads((f.entry / 'ACTIVATION_STATE.json').read_text())['phase'],
                'failed_requires_manual_recovery',
            )

    # 运行参数备份摘要错误必须先拒绝，不能只信任有效服务备份就部分恢复。
    def test_invalid_runtime_backup_refuses_before_restoring_any_service_bytes(self):
        with tempfile.TemporaryDirectory() as temp:
            f = ActivationFixture(temp)
            with f.context():
                backup = f.activate()
            (backup / 'runtime.yaml').write_bytes(b'corrupted')
            installed = (f.units / m.SERVICE).read_bytes()
            before = len(f.calls)
            with f.context(), self.assertRaisesRegex(m.ControlError, '运行参数备份摘要'):
                m.rollback(
                    f.entry, f.home, f.sdk, backup, units=f.units, run=f.run, stop_current=True
                )
            self.assertEqual((f.units / m.SERVICE).read_bytes(), installed)
            self.assertFalse(any('/usr/bin/install' in a for a in f.calls[before:]))


# 读取临时 /proc 树与 status，核对实际 exe/cwd/PID、启动版本和当前安全反馈。
class RunningIdentityTests(unittest.TestCase):
    # 旧版本、历史 PID/状态、错误策略、FAULT、armed 或工作目录不一致均不能通过启用验收。
    def test_proc_identity_and_current_policy_reject_stale_or_wrong_execution(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            entry = root / 'new'
            entry.mkdir()
            home = root / 'home'
            (home / 'FC01_navigation').mkdir(parents=True)
            binary = entry / 'fc01_system_node'
            binary.write_bytes(b'controller')
            proc = root / 'proc'
            (proc / 'sys/kernel/random').mkdir(parents=True)
            (proc / 'sys/kernel/random/boot_id').write_text('boot')
            for pid, parent in ((20, 1), (42, 20)):
                p = proc / str(pid)
                p.mkdir()
                (p / 'status').write_text('PPid:\t' + str(parent) + '\n')
                (p / 'cwd').symlink_to(entry)
            (proc / '42/exe').symlink_to(binary)
            row = dict(
                updated_unix=time.time(),
                state_age_seconds=0.1,
                boot_id='boot',
                entry_version=m.ENTRY_VERSION,
                supervisor_pid=20,
                controller_pid=42,
                state_controller_pid=42,
                feedback_current=True,
                runtime_mode='flight-test',
                config_source_path=str(entry / 'native/config/flight_test.yaml'),
                config_path=str(home / 'FC01_navigation/runtime.yaml'),
                status='WAITING',
                state={
                    'vertical_feedback_velocity_policy': 'diagnostic_only',
                    'phase': 'IDLE',
                    'armed': False,
                    'arming_state_known': True,
                    'status_fresh': True,
                    'land_fresh': True,
                    'landed': True,
                    'at_rest': True,
                },
            )
            status = home / 'FC01_navigation/status.json'
            controller = types.SimpleNamespace(pid=42, uid=os.getuid(), argv=(str(binary),))
            with (
                patch.object(
                    m, 'unit_info', return_value={'MainPID': '20', 'ActiveState': 'active'}
                ),
                patch.object(m, '_controllers', return_value=[controller]),
                patch.object(m, 'retained_agent'),
            ):
                status.write_text(json.dumps(row))
                m.verify_running(entry, home, binary, object(), proc=proc)
                for field, value in (
                    ('entry_version', 'old'),
                    ('feedback_current', False),
                    ('state_controller_pid', 99),
                    ('updated_unix', time.time() - 10),
                ):
                    status.write_text(json.dumps({**row, field: value}))
                    with self.assertRaises(m.ControlError):
                        m.verify_running(entry, home, binary, object(), proc=proc)
                status.write_text(
                    json.dumps({**row, 'state': {'vertical_feedback_velocity_policy': 'blocking'}})
                )
                with self.assertRaisesRegex(m.ControlError, '策略'):
                    m.verify_running(entry, home, binary, object(), proc=proc)
                for change in (
                    {'status': 'FAULT'},
                    {'state': {**row['state'], 'phase': 'SAFETY'}},
                    {'state': {**row['state'], 'land_fresh': False}},
                    {'state': {**row['state'], 'armed': True}},
                ):
                    status.write_text(json.dumps({**row, **change}))
                    with self.assertRaises(m.ControlError):
                        m.verify_running(entry, home, binary, object(), proc=proc)
                status.write_text(json.dumps(row))
                (proc / '42/cwd').unlink()
                (proc / '42/cwd').symlink_to(home)
                with self.assertRaisesRegex(m.ControlError, '工作目录'):
                    m.verify_running(entry, home, binary, object(), proc=proc)


# RC 地面许可同时依赖前进样本、唯一发布源、新鲜时钟和安全档位/摇杆。
class RcProbeTests(unittest.TestCase):
    # 生成时间一致的 manual/switches/flags；throttle=-1 表示最低油门，arm/kill=3 是 PX4 OFF 枚举。
    def values(self, t):
        stamp = int(t * 1e6)
        return dict(
            manual=dict(
                timestamp=stamp,
                timestamp_sample=stamp,
                valid=True,
                data_source=1,
                sticks_moving=False,
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
                throttle=-1.0,
            ),
            switches=dict(
                timestamp=stamp,
                timestamp_sample=stamp,
                arm_switch=3,
                kill_switch=3,
                mode_slot=2,
                return_switch=0,
                loiter_switch=3,
                offboard_switch=3,
            ),
            flags=dict(timestamp=stamp, manual_control_signal_lost=False),
        )

    # 逐个破坏低油门、来源、解锁/Kill/模式及信号状态，证明任一异常都撤销地面许可。
    def test_rc_requires_advancing_unique_fresh_safe_controls(self):
        e = RcGroundEvidence(1)
        for t in (100.0, 100.1):
            for kind, row in self.values(t).items():
                e.observe(kind, row, t, t)
        publishers = {k: 1 for k in e.counts}
        self.assertTrue(e.ready(100.1, 100.1, publishers))
        self.assertFalse(e.ready(100.1, 100.1, {**publishers, 'switches': 2}))
        self.assertFalse(e.ready(103.0, 103.0, publishers))
        for kind, changes in (
            ('manual', {'throttle': 0.0}),
            ('manual', {'data_source': 2}),
            ('switches', {'arm_switch': 1}),
            ('switches', {'kill_switch': 1}),
            ('switches', {'mode_slot': 1}),
            ('flags', {'manual_control_signal_lost': True}),
        ):
            trial = RcGroundEvidence(1)
            for t in (100.0, 100.1):
                for k, row in self.values(t).items():
                    trial.observe(k, {**row, **(changes if k == kind else {})}, t, t)
            self.assertFalse(trial.ready(100.1, 100.1, publishers))

    # 要求已有 Agent 的 probe 不能为了补数据自行启动新进程。
    def test_missing_retained_agent_never_starts_an_agent(self):
        import navigation_probe as probe

        ros = types.ModuleType('rclpy')
        ros.ok = lambda: False
        qos = types.ModuleType('rclpy.qos')
        qos.qos_profile_sensor_data = object()
        msgs = types.ModuleType('px4_msgs.msg')
        msgs.VehicleStatus = object
        msgs.VehicleLandDetected = object
        with (
            tempfile.TemporaryDirectory() as temp,
            patch.dict(sys.modules, {'rclpy': ros, 'rclpy.qos': qos, 'px4_msgs.msg': msgs}),
            patch.object(probe, 'choose_agent', return_value=None),
            patch.object(probe, 'process_list', return_value=[]),
            patch.object(probe, 'uart_owners', return_value=set()),
            patch.object(probe.subprocess, 'Popen') as spawn,
        ):
            with self.assertRaisesRegex(m.ControlError, '既有 Agent'):
                probe.probe(
                    Path(temp),
                    Path(temp) / 'Agent',
                    Path(temp) / 'probe.json',
                    require_existing_agent=True,
                )
            spawn.assert_not_called()
            self.assertFalse(json.loads((Path(temp) / 'probe.json').read_text())['ok'])


# 认证/部署中收到中断后进入安全收尾，后续信号不能打断该恢复。
class InterruptRecoveryTests(unittest.TestCase):
    # 首个信号抛安全收尾请求后，把三个退出信号设为忽略，避免第二次中断破坏状态。
    def test_forwarded_second_signal_cannot_interrupt_safe_recovery(self):
        with patch.object(m.signal, 'signal') as install:
            handler = m._install_interrupt_handlers()
            self.assertEqual(install.call_count, 3)
            with self.assertRaisesRegex(m.ControlError, '安全收尾'):
                handler(m.signal.SIGINT, None)
            self.assertIsNone(handler(m.signal.SIGTERM, None))
            self.assertEqual(install.call_count, 6)
            self.assertTrue(
                all(call.args[1] == m.signal.SIG_IGN for call in install.call_args_list[-3:])
            )


if __name__ == '__main__':
    unittest.main()
