# 状态窗口与监督器诊断的真实 IPC 回归：严格区分当前控制器锁存、历史飞机反馈和连接异常。
# STATE 只是带明确时效的合成样本；服务 active 或 HEIGHT_HOLD 字样都不能自动证明当前飞机可导航。
"""Real IPC/process regressions for current feedback and retained history."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import launcher
import navigation
import navigation_service
import standby


STATE = dict(kind='state', phase='HEIGHT_HOLD', reason='last native feedback',
             status_fresh=True, local_fresh=True, nav_state=14, armed=True,
             relative_height_m=.42, source_timing={
                 name:dict(source_reason='VALID',receipt_reason='VALID',timeout_s=timeout,
                           envelope_age_s=.02,sample_age_s=.02,receipt_age_s=.02,
                           sample_progress_age_s=.02)
                 for name,timeout in (('status',2.5),('land',2.5),('local',.5),
                                      ('attitude',.5),('flags',2.5),('battery',1.))})


# 检查恢复阻塞原因优先级、动作门提示、离地证据以及子进程异常后的历史标签。
class NavigationDiagnosticsTests(unittest.TestCase):
    # 同时呈现 0.5 秒 local 过期与新鲜 at_rest=false，避免只看到接管锁存却不知实际恢复阻塞。
    def test_recovery_explains_expired_sample_and_independent_fresh_rest_blocker(self):
        state=dict(STATE,phase='SAFETY',reason='OPERATOR_TAKEOVER_LATCHED',local_fresh=False,
                   land_fresh=True,landed=True,at_rest=False,armed=False,
                   rc_recovery_block_reason='RC_RECOVERY_QUEUED_EVENT_DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE',
                   source_timing={'local':dict(source_reason='ENVELOPE_EXPIRED',
                       envelope_age_s=.535409,sample_age_s=.536524,timeout_s=.5,rmw_to_callback_s=.49733)})
        row=dict(status='FAULT',reason=state['reason'],supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=.1,feedback_current=False)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('位置/垂直速度消息过期（消息年龄 0.537 秒，阈值 0.500 秒）',view)
        self.assertIn('位置/垂直速度中间件接收至回调间隔 0.497 秒',view)
        self.assertIn('飞控尚未确认静止（at_rest=false）',view)
        self.assertNotIn('飞控仍解锁',view)
        self.assertNotIn('已准备：CH5 手动解锁',view)

    # land/status 过期或 IPC 已断时，旧 armed/接管信息只能作历史，不能显示为当前恢复条件。
    def test_recovery_never_revives_historical_detector_or_takeover_observations(self):
        state=dict(STATE,phase='SAFETY',land_fresh=False,status_fresh=False,
                   landed=False,at_rest=False,armed=True,px4_user_takeover_reported=True)
        row=dict(status='FAULT',supervisor_pid=10,updated_unix=1000,state=state,
                 state_age_seconds=.1,feedback_current=False)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertNotIn('恢复条件快照',view)
        state.update(land_fresh=True,status_fresh=True)
        row['ipc_error']=dict(kind='clean_eof')
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertNotIn('恢复条件快照',view)

    # 新鲜 status 明确报告人工接管时应显示真实阻塞，不能错误归因于消息过期。
    def test_recovery_reports_actual_takeover_with_fresh_status(self):
        state=dict(STATE,phase='SAFETY',px4_user_takeover_reported=True,
                   source_timing={'local':dict(source_reason='VALID',timeout_s=.5)})
        row=dict(status='FAULT',supervisor_pid=10,updated_unix=1000,state=state,
                 state_age_seconds=.1,feedback_current=True)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('飞控仍报告人工接管有效',view)
        self.assertNotIn('消息过期',view)

    # 缺失、字符串、NaN/Inf 时间只能降级为未知说明，渲染不能崩溃或把非有限值直接展示。
    def test_recovery_details_tolerate_missing_and_nonfinite_timing(self):
        state=dict(STATE,phase='SAFETY',source_timing={
            'local':dict(source_reason='SAMPLE_EXPIRED',sample_age_s=float('nan'),
                         timeout_s='unknown',rmw_to_callback_s=float('inf')),
            'attitude':None})
        row=dict(status='FAULT',supervisor_pid=10,updated_unix=1000,state=state,
                 state_age_seconds=.1,feedback_current=True)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('恢复条件快照（0.1 秒前）：位置/垂直速度消息过期',view)
        self.assertNotIn('nan',view)
        self.assertNotIn('inf',view)

    # 展示原恢复快照的累计年龄，不能用“观测”措辞把旧 at_rest/接管变成当前事实。
    def test_recovery_observations_explicitly_retain_their_snapshot_age(self):
        state=dict(STATE,phase='SAFETY',land_fresh=True,at_rest=False,
                   px4_user_takeover_reported=True,source_timing={
                       name:dict(source_reason='VALID',envelope_age_s=2.4,timeout_s=2.5)
                       for name in ('status','land')})
        row=dict(status='FAULT',supervisor_pid=10,updated_unix=1000,state=state,
                 state_age_seconds=.1,feedback_current=True)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1001)
        self.assertIn('恢复条件快照（1.1 秒前）',view)
        self.assertNotIn('恢复条件观测',view)
        self.assertNotIn('已准备：CH5 手动解锁',view)

    # 飞机反馈过期不抹掉当前控制器的 SAFETY 锁存；同时禁止展示实际模式或手动解锁提示。
    def test_current_safety_remains_fault_when_aircraft_feedback_is_stale(self):
        state=dict(STATE,phase='SAFETY',reason='HEIGHT_OR_SPEED_ENVELOPE_EXCEEDED',local_fresh=False,
                   rc_recovery_block_reason='WAIT_NEW_LOCAL_SAMPLE',flight_actions_enabled=True,
                   takeoff_enabled=True,rc_navigation_enabled=True)
        row=dict(status='WAITING',reason='等待飞控新鲜状态和本地位置',supervisor_pid=10,
                 updated_unix=1000,state=state,state_age_seconds=.05,feedback_current=False)
        for displayed in (row,navigation_service.describe_row(row)):
            with self.subTest(described=displayed is not row):
                view=navigation.render_status(displayed,dict(ActiveState='active',MainPID='10'),now=1000)
                self.assertIn('[FAULT]',view)
                self.assertIn('故障锁存（当前控制器）：HEIGHT_OR_SPEED_ENVELOPE_EXCEEDED',view)
                self.assertIn('飞控反馈：已过期或无效；当前飞机状态未知',view)
                self.assertIn('恢复观察（飞控反馈过期）',view)
                self.assertNotIn('实际模式：',view)
                self.assertNotIn('已准备：CH5 手动解锁',view)

    # IPC/服务/PID/boot 变化或快照过期后，旧故障必须标历史，不继承为新实例当前 FAULT。
    def test_old_safety_is_history_after_disconnect_snapshot_expiry_or_process_change(self):
        row=dict(status='FAULT',reason='previous fault',supervisor_pid=10,updated_unix=1000,
                 state=dict(STATE,phase='SAFETY',reason='previous fault'),state_age_seconds=.1,
                 feedback_current=True,controller_pid=2057,state_controller_pid=2057,boot_id='current')
        info=dict(ActiveState='active',MainPID='10',boot_id='current')
        cases=[(dict(ipc_error=dict(kind='clean_eof')),{},1000),
               (dict(state_age_seconds=4),{},1000),(dict(controller_pid=2058),{},1000),
               ({},{},1020),({},dict(MainPID='11'),1000),({},dict(boot_id='next'),1000),
               ({},dict(ActiveState='inactive'),1000)]
        for changes,service_changes,now in cases:
            with self.subTest(changes=changes,service_changes=service_changes,now=now):
                view=navigation.render_status(dict(row,**changes),dict(info,**service_changes),now=now)
                self.assertNotIn('[FAULT]',view)
                self.assertIn('最近故障锁存（历史）：previous fault',view)
                self.assertNotIn('故障锁存（当前控制器）',view)
                self.assertNotIn('实际模式：',view)
                self.assertNotIn('已准备：CH5 手动解锁',view)

    # 旧 READY 即使尚有保存记录，也不能提示用户操作 CH5 解锁。
    def test_expired_ready_feedback_never_prompts_manual_arming(self):
        state=dict(STATE,phase='RC_READY_WAIT_ARM',armed=False,local_fresh=False,
                   flight_actions_enabled=True,takeoff_enabled=True,rc_navigation_enabled=True)
        row=dict(status='READY_WAIT_ARM',reason='previous ready',supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=.1,feedback_current=False)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('[STALE]',view)
        self.assertNotIn('已准备：CH5 手动解锁',view)
        self.assertNotIn('实际模式：',view)

    # RC 和数据健康但飞机未静止时，提示具体地面窗口条件，不泛称遥控器或预检失效。
    def test_preparation_wait_names_ground_blocker_with_healthy_rc_and_data(self):
        state=dict(STATE,phase='IDLE',reason='RC_WAIT_DISARMED_STABILIZED_ARM_OFF_NAV_OFF_GROUND',
                   armed=False,nav_state=15,landed=True,at_rest=False,rc_arm_switch=3,
                   rc_mode_slot=4,rc_navigation_mode_slot=1,rc_link_ready=True,rc_switches_ready=True,
                   rc_axes_safe=True,rc_sticks_moving=False,preflight=True,battery_fresh=True,
                   stationary_seconds=0.,required_ground_seconds=2.,ground_vz_ned=.002,ground_vz_tolerance=.1)
        row=dict(status='WAITING',reason=state['reason'],supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=.1,feedback_current=True)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('准备等待：未确认静止；持续静止不足（0.00/2.00 秒）',view)
        self.assertNotIn('遥控源/链路未就绪',view)
        self.assertNotIn('飞控预检未通过',view)
        self.assertNotIn('已准备：CH5 手动解锁',view)

    # 多个缺证据同时存在应分别说明，不能在说明信息里暗中授予起飞准备。
    def test_preparation_diagnostics_explain_stale_sources_and_preflight_without_admission(self):
        state=dict(STATE,phase='IDLE',reason='BATTERY_PREFLIGHT_REQUIRED',preflight=False,
                   local_fresh=False,source_clock_valid=False,battery_fresh=False)
        row=dict(status='WAITING',reason=state['reason'],supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=.1,feedback_current=False)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('位置/垂直速度反馈过期',view)
        self.assertIn('源消息时间戳过期或无效',view)
        self.assertIn('飞控预检未通过',view)
        self.assertIn('电池反馈未就绪',view)
        self.assertNotIn('已准备：CH5 手动解锁',view)

    # 首故障/最新故障分别保留；Runtime 未决 Command400 比 Core 地面窗口完成更优先。
    def test_recovery_view_preserves_first_fault_and_prioritizes_actual_runtime_block(self):
        state=dict(STATE,phase='SAFETY',armed=False,rc_first_fault_reason='OPERATOR_TAKEOVER_LATCHED',
                   rc_latest_fault_reason='DDS_STATUS_LAND_OR_FAILSAFE_STALE',
                   rc_recovery_block_reason='GROUND_WINDOW_READY',rc_reprepare_block_reason='COMMAND_400_UNRESOLVED',
                   rc_recovery_elapsed_s=1.25,rc_recovery_required_s=2.,rc_recovery_status_samples=8,
                   rc_recovery_land_samples=4,rc_recovery_local_samples=12,rc_recovery_ready=True,
                   rc_recovery_event_blocked=False,rc_fault_recoverable=True)
        row=dict(status='FAULT',reason='first fault',supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=.1,feedback_current=True)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('[FAULT]',view)
        self.assertIn('首次故障 OPERATOR_TAKEOVER_LATCHED',view)
        self.assertIn('最新故障 DDS_STATUS_LAND_OR_FAILSAFE_STALE',view)
        self.assertIn('恢复等待原因：COMMAND_400_UNRESOLVED',view)
        self.assertIn('1.25/2.00 秒',view)
        self.assertIn('status / land / local：8 / 4 / 12',view)
        self.assertNotIn('已准备：CH5 手动解锁',view)

    # Runtime 没有具体原因时使用 Core 原因；过期恢复记录及非有限窗口必须正确标注。
    def test_recovery_view_uses_core_block_when_runtime_has_none_and_labels_stale_history(self):
        state=dict(STATE,phase='SAFETY',rc_recovery_block_reason='WAIT_NEW_LAND_SAMPLE',
                   rc_reprepare_block_reason='',rc_recovery_elapsed_s=float('nan'),rc_recovery_required_s=2.)
        row=dict(status='FAULT',reason='old fault',supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=20,feedback_current=False)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('最近恢复观察（历史）',view)
        self.assertIn('恢复等待原因：WAIT_NEW_LAND_SAMPLE',view)
        self.assertIn('恢复地面窗口：未知',view)
        self.assertNotIn('nan',view)
        self.assertNotIn('已准备：CH5 手动解锁',view)

    # 笼统重启未满足不得盖过具体 WAIT_GROUND_DWELL，正常新 READY 不应残留旧恢复面板。
    def test_recovery_view_shows_specific_core_wait_and_omits_normal_ready_without_fault(self):
        state=dict(STATE,phase='SAFETY',rc_recovery_block_reason='WAIT_GROUND_DWELL',
                   rc_reprepare_block_reason='RC_CORE_RESTART_CONDITIONS_NOT_MET')
        row=dict(status='FAULT',reason='fault',supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=.1,feedback_current=True)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('恢复等待原因：WAIT_GROUND_DWELL',view)
        self.assertNotIn('RC_CORE_RESTART_CONDITIONS_NOT_MET',view)
        state.update(phase='RC_READY_WAIT_ARM',rc_first_fault_reason='',rc_latest_fault_reason='',
                     rc_recovery_window_active=False)
        view=navigation.render_status(dict(row,status='READY_WAIT_ARM'),dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertNotIn('恢复观察',view)
        self.assertNotIn('恢复等待原因',view)

    # 原生动作/起飞门任一关闭时，只能显示内部准备阶段，不能被监督器乐观角色元数据提升。
    def test_closed_native_action_gate_labels_preparation_as_diagnostic(self):
        for closed in ('flight_actions_enabled', 'takeoff_enabled'):
            with self.subTest(closed=closed):
                state=dict(STATE,phase='RC_READY_WAIT_ARM',armed=False,
                           reason='RC_READY_WAIT_CH5_MANUAL_ARM',flight_enabled=False,
                           arm_enabled=False,flight_actions_enabled=True,takeoff_enabled=True,
                           rc_navigation_enabled=True)
                state[closed]=False
                row=dict(status='CONNECTED',reason='transport okay',supervisor_pid=10,
                         updated_unix=1000,state=state,state_age_seconds=.1,feedback_current=True,
                         flight_actions_enabled=True,rc_navigation_enabled=True)
                described=navigation_service.describe_row(row)
                self.assertEqual(described['status'],'DIAGNOSTIC')
                self.assertIs(described['state'],state)
                self.assertEqual(state['phase'],'RC_READY_WAIT_ARM')
                self.assertEqual(state['reason'],'RC_READY_WAIT_CH5_MANUAL_ARM')
                # The view also protects against a previously written READY
                # label and optimistic supervisor role metadata.
                for displayed in (described,dict(row,status='READY_WAIT_ARM')):
                    view=navigation.render_status(displayed,dict(ActiveState='active',MainPID='10'),now=1000)
                    self.assertIn('[DIAGNOSTIC]',view)
                    self.assertIn(closed+'=false',view)
                    self.assertIn('内部阶段：RC_READY_WAIT_ARM',view)
                    self.assertIn('禁止起飞和导航',view)
                    self.assertNotIn('已准备：CH5 手动解锁',view)

    # arm_enabled=false 只禁止程序解锁，不能误禁已经具备其他合同的人工 RC 解锁路线。
    def test_program_arm_disabled_does_not_disable_valid_manual_rc_guidance(self):
        state=dict(STATE,phase='RC_READY_WAIT_ARM',armed=False,
                   reason='RC_READY_WAIT_CH5_MANUAL_ARM',flight_enabled=False,
                   arm_enabled=False,flight_actions_enabled=True,takeoff_enabled=True,
                   rc_navigation_enabled=True)
        row=dict(status='CONNECTED',reason='transport okay',supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=.1,feedback_current=True)
        described=navigation_service.describe_row(row)
        self.assertEqual(described['status'],'READY_WAIT_ARM')
        view=navigation.render_status(described,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('已准备：CH5 手动解锁',view)
        self.assertIn('arm_enabled=false（程序解锁）',view)
        self.assertNotIn('[DIAGNOSTIC]',view)

    # 动作门缺失须未知；上层 status 或角色声明不能补成 true。
    def test_unknown_native_gates_cannot_prompt_manual_arming(self):
        state=dict(STATE,phase='RC_READY_WAIT_ARM',armed=False)
        row=dict(status='READY_WAIT_ARM',reason='ready',supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=.1,feedback_current=True,
                 flight_actions_enabled=True,rc_navigation_enabled=True)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('flight_actions_enabled=未知',view)
        self.assertIn('尚未确认飞行动作许可',view)
        self.assertNotIn('已准备：CH5 手动解锁',view)

    # 诊断模式不能遮盖当前控制器 FAULT，也不能把历史门状态提升为当前诊断 READY。
    def test_diagnostic_gates_do_not_hide_fault_or_revive_historical_feedback(self):
        state=dict(STATE,phase='SAFETY',reason='CONTROL_LOOP_STALL',
                   flight_actions_enabled=False,takeoff_enabled=False)
        row=dict(status='CONNECTED',reason='transport okay',supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=.1,feedback_current=True)
        fault=navigation_service.describe_row(row)
        self.assertEqual(fault['status'],'FAULT')
        self.assertIn('[FAULT]',navigation.render_status(fault,dict(ActiveState='active',MainPID='10'),now=1000))
        state['phase']='RC_READY_WAIT_ARM'
        historical=dict(row,status='WAITING',reason='native disconnected',feedback_current=False)
        self.assertEqual(navigation_service.describe_row(historical)['status'],'WAITING')
        view=navigation.render_status(historical,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('[WAITING]',view)
        self.assertIn('最近动作门（历史）',view)
        self.assertNotIn('[DIAGNOSTIC]',view)
        self.assertNotIn('已准备：CH5 手动解锁',view)

    # 未取得离地反馈时，即使 phase 写 HEIGHT_HOLD 也应明确离地未确认和导航未允许。
    def test_explicit_unconfirmed_liftoff_never_displays_height_hold_as_ready(self):
        state=dict(STATE,takeoff_stage='LIFTOFF_UNCONFIRMED',liftoff_confirmed=False,
                   target_height_settled=False,navigation_allowed=False,
                   liftoff_unconfirmed_reason='WAIT_NEW_LOCAL_MOTION_SAMPLES')
        row=dict(status='HEIGHT_HOLD',reason='previous phase',supervisor_pid=10,
                 updated_unix=1000,state=state,state_age_seconds=.1,feedback_current=True)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('离地未确认',view)
        self.assertIn('WAIT_NEW_LOCAL_MOTION_SAMPLES',view)
        self.assertIn('导航未允许',view)
        self.assertNotIn('正在定高',view)

    # 本地 FRD 推力 z=-0.50 仅表示向上推力提交，不能据此声称 PX4 已采用。
    def test_view_distinguishes_local_thrust_commit_from_px4_adoption(self):
        state=dict(STATE,takeoff_stage='TARGET_SETTLED',liftoff_confirmed=True,
                   target_height_settled=True,navigation_allowed=True,
                   target_local_commits=25,last_target_local_thrust_z=-.50,
                   px4_thrust_adoption='UNOBSERVED')
        row=dict(status='HEIGHT_HOLD',reason='hold',supervisor_pid=10,updated_unix=1000,
                 state=state,state_age_seconds=.1,feedback_current=True)
        view=navigation.render_status(row,dict(ActiveState='active',MainPID='10'),now=1000)
        self.assertIn('本地推力',view)
        self.assertIn('-0.5',view)
        self.assertIn('PX4 采用尚未取证',view)

    # 启动临时 Python socket peer；可分别制造非法尾部、无输出存活、信号退出及非零退出。
    def peer(self, *, tail=b'', exit_code=0, exit_signal=None, silent_seconds=.08,
             keep_open=False, exit_on_stop=False):
        a, b = socket.socketpair()
        self.addCleanup(a.close)
        code = 'import os,signal,socket,sys,time;'
        code += ('signal.signal(signal.SIGTERM,lambda *_:sys.exit(0));' if exit_on_stop else
                 'signal.signal(signal.SIGTERM,lambda *_:None);')
        code += ('s=socket.socket(fileno=int(sys.argv[1]));'
                's.sendall(bytes.fromhex(sys.argv[2]));')
        if exit_signal is not None:
            code += 'os.kill(os.getpid(),int(sys.argv[3]));'
        else:
            if not keep_open and not silent_seconds > 3:
                code += 's.shutdown(socket.SHUT_WR);'
            code += 'time.sleep(float(sys.argv[4]));sys.exit(int(sys.argv[3]));'
        wire = (json.dumps(STATE)+'\n').encode()+tail
        node = subprocess.Popen([sys.executable, '-c', code, str(b.fileno()), wire.hex(),
                                 str(int(exit_signal) if exit_signal is not None else exit_code), str(silent_seconds)],
                                pass_fds=(b.fileno(),))
        b.close()
        def cleanup():
            if node.poll() is None:
                node.kill()
            node.wait(timeout=3)
        self.addCleanup(cleanup)
        return launcher.Channel(a), node

    # 异常退出不能丢掉此前完整状态及 IPC 错误，两类证据都要保留。
    def test_complete_feedback_and_ipc_failure_survive_nonzero_native_exit(self):
        channel, node = self.peer(tail=b'{"kind":', exit_code=7)
        rows = []
        state, _, _ = standby.monitor(channel, node, threading.Event(), rows.append)
        self.assertEqual(state.get('relative_height_m'), .42)
        self.assertEqual(rows[-1].get('ipc_error', {}).get('kind'), 'truncated_json')
        self.assertEqual(rows[-1].get('controller_returncode'), 7)
        self.assertEqual(rows[-1].get('controller_exit_code'), 7)
        self.assertIsNone(rows[-1].get('controller_exit_signal'))
        self.assertIs(rows[-1].get('feedback_current'), False)

    # EOF 与进程 returncode 是两条独立证据，正常退出码不等于通道没有断流。
    def test_clean_eof_is_recorded_separately_from_normal_native_exit(self):
        channel, node = self.peer(exit_code=0)
        rows = []
        standby.monitor(channel, node, threading.Event(), rows.append)
        self.assertEqual(rows[-1].get('ipc_error', {}).get('kind'), 'clean_eof')
        self.assertEqual(rows[-1].get('controller_returncode'), 0)
        self.assertEqual(rows[-1].get('controller_exit_code'), 0)

    # 负返回码须解析成对应信号名，不能只写笼统“控制器退出”。
    def test_native_signal_exit_keeps_signal_identity(self):
        channel, node = self.peer(exit_signal=signal.SIGUSR1)
        rows = []
        standby.monitor(channel, node, threading.Event(), rows.append)
        self.assertEqual(rows[-1].get('controller_returncode'), -signal.SIGUSR1)
        self.assertEqual(rows[-1].get('controller_exit_signal'), signal.SIGUSR1)
        self.assertEqual(rows[-1].get('controller_exit_signal_name'), 'SIGUSR1')
        self.assertIsNone(rows[-1].get('controller_exit_code'))

    # 合法状态后紧随非法尾部时及时进入停止恢复，不能等下一次 select 永久卡住。
    def test_invalid_tail_stops_live_peer_without_waiting_for_another_read_event(self):
        channel, node = self.peer(tail=b'[]\n', exit_code=9, silent_seconds=.5,
                                  keep_open=True, exit_on_stop=True)
        rows = []
        state, _, _ = standby.monitor(channel, node, threading.Event(), rows.append)
        self.assertEqual(state.get('relative_height_m'), .42)
        self.assertEqual(rows[-1].get('ipc_error', {}).get('kind'), 'invalid_message')
        self.assertEqual(node.returncode, 0, 'native must receive the graceful stop before its timeout')

    # 原生进程仍活着但不发送数据，不得续期飞机反馈或维持当前飞行状态显示。
    def test_silent_live_controller_cannot_refresh_aircraft_feedback(self):
        channel, node = self.peer(silent_seconds=3.25)
        rows = []
        standby.monitor(channel, node, threading.Event(), rows.append)
        live_stale = [row for row in rows if row['status'] == 'WAITING'
                      and (row.get('state_age_seconds') or 0) > 3]
        self.assertTrue(live_stale, rows)
        self.assertIs(live_stale[0].get('feedback_current'), False)
        self.assertEqual(live_stale[0]['state']['relative_height_m'], .42)

    # 重试期间保留故障年龄、IPC 原因和退出码，不用新监督快照刷新历史飞机状态。
    def test_retry_retains_history_age_ipc_error_and_native_exit(self):
        with tempfile.TemporaryDirectory() as folder:
            home = Path(folder)
            a, b = socket.socketpair()
            self.addCleanup(a.close)
            self.addCleanup(b.close)
            binary = home/'native_fixture'
            wire = (json.dumps(STATE)+'\n').encode().hex()
            binary.write_text('#!/usr/bin/env python3\nimport signal,socket,sys,time\n'
                              'signal.signal(signal.SIGTERM,lambda *_:None)\n'
                              's=socket.socket(fileno=int(sys.argv[-1]))\n'
                              f's.sendall(bytes.fromhex({wire!r}))\n'
                              's.shutdown(socket.SHUT_WR)\ntime.sleep(.08)\nsys.exit(7)\n')
            binary.chmod(0o700)
            retries = []
            class StopOnRetries(threading.Event):
                def wait(self, timeout=None):
                    if timeout == 5:
                        end=time.monotonic()+1
                        while True:
                            try:row=json.loads((home/'state/status.json').read_text())
                            except (OSError,ValueError):row={}
                            newer=not retries or row.get('updated_unix',0)>retries[-1].get('updated_unix',0)
                            if newer and row.get('controller_returncode')==7 and 'retry_in_seconds' in row and (attempts[0]<=1 or 'UART disappeared' in row.get('reason','')):
                                break
                            if time.monotonic()>=end:break
                            time.sleep(.002)
                        retries.append(row)
                        if len(retries) == 8:
                            self.set()
                        else:
                            time.sleep(.03)
                        return self.is_set()
                    return super().wait(timeout)
            existing = launcher.Process(os.getpid(), os.getuid(), ('fixture-agent',))
            real_popen, real_read = subprocess.Popen, Path.read_bytes
            attempts = [0]
            def preflight(*_):
                attempts[0] += 1
                if attempts[0] > 1:
                    raise launcher.ControlError('UART disappeared during reconnect')
                return binary, existing
            def spawn(argv, **kwargs):
                return real_popen([*argv, str(b.fileno())], pass_fds=(b.fileno(),), **kwargs)
            def read_bytes(path):
                if path == Path('/proc', str(existing.pid), 'cmdline'):
                    return b'fixture-agent\0'
                return real_read(path)
            with patch.object(Path, 'home', return_value=home), \
                 patch.object(standby, 'preflight', side_effect=preflight), \
                 patch.object(standby, 'validate', return_value={}), \
                 patch.object(standby.subprocess, 'Popen', side_effect=spawn), \
                 patch.object(standby, 'connect_controller', return_value=launcher.Channel(a)), \
                 patch.object(Path, 'read_bytes', new=read_bytes):
                standby.supervise(home/'project', home/'agent', home/'state', StopOnRetries())
            self.assertEqual(len(retries), 8)
            for row in retries:
                self.assertEqual(row.get('state', {}).get('relative_height_m'), .42)
                self.assertIs(row.get('feedback_current'), False)
                self.assertEqual(row.get('ipc_error', {}).get('kind'), 'clean_eof')
                self.assertEqual(row.get('controller_returncode'), 7)
                self.assertIn('7', row['reason'])
            self.assertGreater(retries[1]['state_age_seconds'], retries[0]['state_age_seconds'])
            self.assertIn('UART disappeared during reconnect', retries[-1]['reason'])
            final = json.loads((home/'state/status.json').read_text())
            self.assertEqual(final['status'], 'STOPPED')
            self.assertEqual(final.get('state', {}).get('relative_height_m'), .42)
            self.assertIs(final.get('feedback_current'), False)

    # 监督器刚写文件不代表其中飞机状态新鲜，必须按实际状态年龄标历史。
    def test_view_labels_history_even_when_supervisor_snapshot_is_fresh(self):
        row = dict(status='WAITING', reason='retry after disconnect', supervisor_pid=10,
                   updated_unix=1000, state=STATE, state_age_seconds=20, feedback_current=False)
        view = navigation.render_status(row, dict(ActiveState='active', MainPID='10'), now=1001)
        self.assertIn('历史', view)
        self.assertIn('Offboard', view)
        self.assertIn('0.42 m', view)
        self.assertIn('当前飞机状态未知', view)
        self.assertNotIn('实际模式：', view)
        self.assertIn('21.0', view)

    # 读取保存快照后经过的时间也消耗 Topic TTL，不能冻结在写盘时年龄。
    def test_view_adds_snapshot_elapsed_time_to_feedback_age(self):
        row = dict(status='HEIGHT_HOLD', reason='hold', supervisor_pid=10, updated_unix=1000,
                   state=STATE, state_age_seconds=2.5, feedback_current=True)
        view = navigation.render_status(row, dict(ActiveState='active', MainPID='10'), now=1001)
        self.assertIn('历史', view)
        self.assertNotIn('实际模式：', view)
        self.assertNotIn('正在定高', view)
        self.assertIn('[STALE]', view)

    # 高度显示须同时满足原生新鲜标志；未知相对高度保留具体原因。
    def test_view_requires_native_fresh_flags_and_explains_missing_height(self):
        row = dict(status='WAITING', reason='feedback not ready', supervisor_pid=10,
                   updated_unix=1000, state=dict(STATE, local_fresh=False, relative_height_m=None),
                   state_age_seconds=.1, feedback_current=True)
        view = navigation.render_status(row, dict(ActiveState='active', MainPID='10'), now=1001)
        self.assertNotIn('实际模式：', view)
        self.assertIn('relative_height_m', view)

    # 导航期望/观测档位和原生高度拒绝原因直接进入诊断，便于现场追踪。
    def test_view_shows_native_height_reason_and_expected_observed_slots(self):
        row = dict(status='WAITING', reason='ground reference pending', supervisor_pid=10,
                   updated_unix=1000, state=dict(STATE, relative_height_m=None,
                       height_unavailable_reason='REFERENCE_NOT_READY',
                       rc_navigation_mode_slot=1, rc_mode_slot=2),
                   state_age_seconds=.1, feedback_current=True)
        view = navigation.render_status(row, dict(ActiveState='active', MainPID='10'), now=1001)
        self.assertIn('REFERENCE_NOT_READY', view)
        self.assertIn('期望 1 / 观测 2', view)

    # 仅当累计年龄和各原生 fresh 位均有效，才允许把模式/高度作为当前反馈展示。
    def test_view_shows_current_feedback_only_with_valid_age_and_native_flags(self):
        row = dict(status='HEIGHT_HOLD', reason='hold', supervisor_pid=10, updated_unix=1000,
                   state=STATE, state_age_seconds=.1, feedback_current=True)
        view = navigation.render_status(row, dict(ActiveState='active', MainPID='10'), now=1000.1)
        self.assertIn('实际模式：Offboard', view)
        self.assertIn('反馈年龄：0.2 秒', view)
        self.assertIn('正在定高', view)
        self.assertNotIn('历史', view)


if __name__ == '__main__':
    unittest.main()
