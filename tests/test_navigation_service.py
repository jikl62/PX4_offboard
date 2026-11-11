# 正式导航服务的身份、配置和显示合同回归：默认 1.5 m、人工 RC 解锁、视觉关闭。
# 少量真实子进程只运行临时 peer；服务与 Agent 查找均注入，不能把此测试写成真实 PX4 验收。
import importlib
import io
import contextlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import yaml

ROOT=Path(__file__).resolve().parents[1]

# 分别检查制品标识、显式模式、原生动作门、当前实例身份及监督器优雅恢复。
class NavigationServiceTests(unittest.TestCase):
    # 加载当前后台入口，不允许测试自动回退到旧版服务模块。
    def api(self):
        self.assertTrue((ROOT/'navigation_service.py').exists(),'navigation background controller missing')
        return importlib.import_module('navigation_service')

    # 依次加入历史生命周期标记仍应拒绝，只有当前完整 V5 标识才可选中。
    def test_old_binary_without_lifecycle_marker_is_not_accepted(self):
        m = self.api()
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            binary = root/'fc01_system_node'
            markers = m.MARKER+m.TAKEOFF_MARKER+m.SAFETY_MARKER
            binary.write_bytes(markers)
            with patch.object(m, 'controller_path', return_value=binary), patch.object(m, 'verify_binary'):
                with self.assertRaisesRegex(m.ControlError, '生命周期'):
                    m.check_controller(root)
                binary.write_bytes(markers+b'FC01_SAFE_LIFECYCLE_V1_20261109')
                with self.assertRaisesRegex(m.ControlError, '生命周期'):
                    m.check_controller(root)
                binary.write_bytes(markers+b'FC01_VERTICAL_DIAGNOSTIC_V1_20261110')
                with self.assertRaisesRegex(m.ControlError, '生命周期'):
                    m.check_controller(root)
                binary.write_bytes(markers+b'FC01_RECOVERY_DIAGNOSTIC_V1_20261110')
                with self.assertRaisesRegex(m.ControlError, '生命周期'):
                    m.check_controller(root)
                binary.write_bytes(markers+b'FC01_RECOVERY_DIAGNOSTIC_V4_20261110')
                with self.assertRaisesRegex(m.ControlError, '生命周期'):
                    m.check_controller(root)
                binary.write_bytes(markers+m.LIFECYCLE_MARKER)
                self.assertEqual(m.check_controller(root), binary)

    # 源包版本、目录名、入口版本和 C++ 内嵌标识必须一致，避免源码新而二进制旧。
    def test_release_entry_and_native_controller_share_the_current_identity(self):
        m=self.api();bundle=importlib.import_module('build_navigation_bundle')
        self.assertEqual(m.ENTRY_VERSION,'20261110.7')
        self.assertEqual(bundle.RELEASE_VERSION,m.ENTRY_VERSION)
        self.assertEqual(bundle.RELEASE_NAME,'FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5')
        self.assertEqual(bundle.DEFAULT_DIRECTORY,bundle.RELEASE_NAME)
        self.assertEqual(m.LIFECYCLE_MARKER,b'FC01_RECOVERY_DIAGNOSTIC_V5_20261110')
        self.assertEqual(bundle.CONTROLLER_BUILD,m.LIFECYCLE_MARKER.decode())
        native=(ROOT/'native/src/jikl_fc01_system/src/fc01_system_node.cpp').read_text()
        self.assertIn(bundle.CONTROLLER_BUILD,native)
        self.assertNotIn('FC01_RECOVERY_DIAGNOSTIC_V4_20261110',native)

    # 候选包私有 ELF 可独立于 SDK 自带控制器校验；损坏私有 ELF 不能被 SDK 旧程序替代。
    def test_private_controller_does_not_require_the_sdk_controller_binary(self):
        import launcher
        import standby
        import types
        self.api()
        msgs=types.ModuleType('px4_msgs.msg')
        for name in ('BatteryStatus','VehicleAttitude','VehicleLocalPosition','VehicleStatus','VehicleLandDetected','FailsafeFlags'):
            setattr(msgs,name,type(name,(),{}))
        px4=types.ModuleType('px4_msgs');px4.msg=msgs
        rclpy=types.ModuleType('rclpy');support=types.ModuleType('rclpy.type_support')
        support.check_for_type_support=lambda _:None
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'config').mkdir()
            (root/'config/attitude_altitude.yaml').write_text((ROOT/'tests/fixtures/base.yaml').read_text())
            binary=root/'private/fc01_system_node';binary.parent.mkdir()
            data=bytearray(64);data[:6]=b'\x7fELF\x02\x01';data[18:20]=(183).to_bytes(2,'little')
            binary.write_bytes(bytes(data)+b''.join(launcher.BINARY_MARKERS));binary.chmod(0o700)
            agent=root/'MicroXRCEAgent';agent.write_text('#!/bin/sh\n');agent.chmod(0o700)
            with patch.dict(sys.modules,{'px4_msgs':px4,'px4_msgs.msg':msgs,'rclpy':rclpy,'rclpy.type_support':support}),\
                 patch.object(launcher.platform,'machine',return_value='aarch64'),\
                 patch.object(launcher,'process_list',return_value=[]),patch.object(launcher,'uart_owners',return_value=set()),\
                 patch.object(standby.shutil,'which',return_value='/usr/bin/fuser'),\
                 patch.dict(os.environ,ROS_DOMAIN_ID='0',ROS_LOCALHOST_ONLY='0'):
                selected,_=launcher.preflight(root,agent,'/dev/null',controller=binary)
                config=standby.validate(root,agent,controller=binary)
                self.assertEqual(selected,binary)
                self.assertEqual(config['fc01_system']['ros__parameters']['profile'],'attitude_altitude')
                binary.write_bytes(b'invalid')
                with self.assertRaises(launcher.ControlError):launcher.preflight(root,agent,'/dev/null',controller=binary)
                with self.assertRaises(launcher.ControlError):standby.validate(root,agent,controller=binary)
                # Installation prepares YAML before compiling its controller.
                self.assertEqual(standby.validate(root,agent)['fc01_system']['ros__parameters']['profile'],'attitude_altitude')

    # 只打开人工 RC 导航所需门，arm_enabled/通用 flight 门保持关闭，推力与监测参数沿用输入。
    def test_navigation_profile_keeps_program_arm_disabled_and_numeric_limits(self):
        m=self.api();base=yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text())
        result=m.navigation_profile(base)['fc01_system']['ros__parameters']
        self.assertTrue(result['rc_navigation_enabled'])
        self.assertTrue(result['flight_actions_enabled'] and result['takeoff_enabled'])
        self.assertFalse(result['arm_enabled'] or result['flight_enabled'] or result['bench_arm_only_enabled'])
        self.assertTrue(result['rc_takeover_confirmed'] and result['flight_parameters_confirmed'])
        self.assertFalse(result['autonomous_failsafe_confirmed'])
        self.assertEqual(result['rc_target_height'],1.5)
        self.assertEqual(result['rc_navigation_mode_slot'],6)
        self.assertEqual(result['csv_path'],'')
        for name in ('aa_hover_thrust','aa_takeoff_reserve_thrust','aa_max_tilt_deg','aa_feedback_rate_tolerance_mps','aa_feedback_dwell_seconds'):
            self.assertEqual(result[name],base['fc01_system']['ros__parameters'][name])

    # 诊断配置明确清零动作、RC 权限、视觉和确认门；1.5 m 仅是保留目标值。
    def test_diagnostic_mode_closes_every_action_authority_and_visual_gate(self):
        m=self.api();base=yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text())
        p=m.diagnostic_profile(base)['fc01_system']['ros__parameters']
        for key in ('flight_enabled','flight_actions_enabled','arm_enabled','takeoff_enabled',
                    'flight_parameters_confirmed','rc_navigation_enabled','rc_takeover_confirmed',
                    'autonomous_failsafe_confirmed','bench_arm_only_enabled','visual_navigation_enabled'):
            self.assertIs(p[key],False,key)
        self.assertEqual(p['rc_target_height'],1.5)

    # 仓库候选 YAML 本身必须全关门，不能因文件名 flight_test 就默认有控制权限。
    def test_shipped_mode_candidates_start_with_closed_gates(self):
        for name in ('diagnostic.yaml','flight_test.yaml'):
            with self.subTest(name=name):
                p=yaml.safe_load((ROOT/'native/config'/name).read_text())['fc01_system']['ros__parameters']
                self.assertFalse(p['flight_actions_enabled'])
                self.assertFalse(p['arm_enabled'])
                self.assertFalse(p['takeoff_enabled'])
                self.assertFalse(p['rc_navigation_enabled'])
                self.assertFalse(p['visual_navigation_enabled'])
                self.assertEqual(p['rc_target_height'],1.5)

    # 加载当前候选配置，运行/源路径和模式元数据必须指向本次真实会话。
    def test_load_profile_uses_bundle_candidate_and_writes_mode_metadata(self):
        m=self.api();base=yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text());seen={}
        def validate(root,agent,**kwargs):
            seen.update(kwargs);return base
        with tempfile.TemporaryDirectory() as d,patch.object(m.standby,'validate',side_effect=validate):
            entry=Path(d);out=entry/'runtime.yaml'
            p=m.load_profile(entry,entry/'agent',entry,m.DIAGNOSTIC,config_path=out)
            params=p['fc01_system']['ros__parameters']
            self.assertEqual(seen['config_path'],entry/'native/config/diagnostic.yaml')
            self.assertEqual(params['fc01_runtime_mode'],'diagnostic')
            self.assertEqual(Path(params['fc01_config_path']),out.resolve())
            self.assertEqual(Path(params['fc01_config_source_path']),seen['config_path'].resolve())

    # 缺少 --confirm-flight-test 时 CLI 在实际启动动作前以参数错误退出。
    def test_flight_test_requires_explicit_confirmation(self):
        m=self.api()
        with self.assertRaises(SystemExit) as error:
            m.main(['run','--mode','flight-test'])
        self.assertEqual(error.exception.code,2)

    # 只接受整数 1..6；bool、浮点、字符串和越界不能混入 PX4 档位合同。
    def test_mode_slot_is_configurable_and_rejects_invalid_values(self):
        m=self.api();base=yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text())
        for slot in range(1,7):
            with self.subTest(slot=slot):
                p=m.navigation_profile(base,mode_slot=slot)['fc01_system']['ros__parameters']
                self.assertEqual(p['rc_navigation_mode_slot'],slot)
                self.assertFalse(p['arm_enabled'])
        for slot in (0,7,-1,True,1.0,'1',None):
            with self.subTest(invalid=slot),self.assertRaises(m.ControlError):
                m.navigation_profile(base,mode_slot=slot)

    # 设置显式档位时覆盖，省略时保留基线档位，不能静默恢复硬编码默认。
    def test_load_profile_uses_settings_slot_and_preserves_base_when_omitted(self):
        m=self.api();base=yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text())
        base['fc01_system']['ros__parameters']['rc_navigation_mode_slot']=2
        with tempfile.TemporaryDirectory() as d,patch.object(m.standby,'validate',return_value=base):
            entry=Path(d);path=entry/'navigation_settings.json'
            path.write_text(json.dumps({'target_height_m':.5,'navigation_mode_slot':1}))
            self.assertEqual(m.load_profile(entry,entry/'agent',entry)['fc01_system']['ros__parameters']['rc_navigation_mode_slot'],1)
            path.write_text(json.dumps({'target_height_m':.5}))
            self.assertEqual(m.load_profile(entry,entry/'agent',entry)['fc01_system']['ros__parameters']['rc_navigation_mode_slot'],2)

    # 断流/重试历史里的 SAFETY/READY/HEIGHT_HOLD 不能盖过当前传输故障。
    def test_historical_phase_cannot_override_retry_or_channel_failure(self):
        m=self.api()
        for phase in ('SAFETY','RC_READY_WAIT_ARM','HEIGHT_HOLD'):
            row=dict(status='WAITING',reason='truncated_json; native exit 2',feedback_current=False,
                     state=dict(phase=phase,reason='previous flight reason'))
            described=m.describe_row(row)
            self.assertEqual(described['status'],'WAITING')
            self.assertEqual(described['reason'],'truncated_json; native exit 2')

    # 当前原生 SAFETY 优先于 CONNECTED；准备等待和高度保持也须分别表达。
    def test_native_fault_overrides_connected_and_ready_is_distinct(self):
        m=self.api()
        common=dict(status='CONNECTED',reason='transport okay',state_age_seconds=.1,feedback_current=True,
                    state=dict(phase='SAFETY',reason='VERTICAL_VELOCITY_INCONSISTENT'))
        row=m.describe_row(common)
        self.assertEqual(row['status'],'FAULT')
        self.assertIn('VERTICAL_VELOCITY_INCONSISTENT',row['reason'])
        common['state']=dict(phase='RC_READY_WAIT_ARM',reason='RC_READY_WAIT_CH5_MANUAL_ARM')
        self.assertEqual(m.describe_row(common)['status'],'READY_WAIT_ARM')
        common['state']=dict(phase='HEIGHT_HOLD',reason='HEIGHT_HOLD')
        self.assertEqual(m.describe_row(common)['status'],'HEIGHT_HOLD')

    # 控制器自身仍当前时，飞机反馈过期不能清除它的故障锁存。
    def test_current_native_fault_survives_expired_aircraft_feedback(self):
        m=self.api()
        state=dict(phase='SAFETY',reason='HEIGHT_OR_SPEED_ENVELOPE_EXCEEDED',
                   status_fresh=True,local_fresh=False,controller_pid=2057)
        row=dict(status='WAITING',reason='等待飞控新鲜状态和本地位置',feedback_current=False,
                 state=state,state_age_seconds=.05,controller_pid=2057,state_controller_pid=2057)
        described=m.describe_row(row)
        self.assertEqual(described['status'],'FAULT')
        self.assertEqual(described['reason'],state['reason'])
        self.assertIs(described['controller_state_current'],True)
        self.assertIs(described['fault_latched'],True)
        self.assertIs(described['feedback_current'],False)
        self.assertEqual(row['status'],'WAITING')

    # 旧 PID、过期、EOF、退出或重试时，历史 SAFETY 不再代表当前实例锁存。
    def test_old_or_disconnected_safety_cannot_override_transport_status(self):
        m=self.api()
        base=dict(status='WAITING',reason='native connection unavailable',feedback_current=False,
                  state=dict(phase='SAFETY',reason='previous fault',controller_pid=2057),
                  state_age_seconds=.05,controller_pid=2057,state_controller_pid=2057)
        cases=[dict(state_age_seconds=3.01),dict(state_age_seconds=None),dict(state_age_seconds=float('nan')),
               dict(ipc_error=dict(kind='clean_eof')),dict(controller_returncode=0),
               dict(retry_in_seconds=5),dict(controller_pid=2058),dict(state_controller_pid=2056),
               dict(status='STOPPING'),dict(status='STOPPED')]
        for changes in cases:
            with self.subTest(changes=changes):
                row=dict(base,**changes)
                described=m.describe_row(row)
                self.assertEqual(described['status'],row['status'])
                self.assertEqual(described['reason'],row['reason'])
                self.assertIs(described['controller_state_current'],False)
                self.assertIs(described['fault_latched'],False)

    # 只收到 READY 字样但缺 feedback_current 合同，不能提升为可人工解锁。
    def test_unknown_aircraft_feedback_cannot_become_ready(self):
        m=self.api()
        row=dict(status='CONNECTED',reason='IPC connected',state_age_seconds=.1,
                 state=dict(phase='RC_READY_WAIT_ARM',reason='RC_READY_WAIT_CH5_MANUAL_ARM'))
        self.assertEqual(m.describe_row(row)['status'],'CONNECTED')

    # 真实 peer 收到的命令只能 STATUS；停止请求后继续等下降完成，既有 Agent 保留。
    def test_service_runs_private_native_with_status_only_and_graceful_recovery(self):
        m=self.api();import standby
        from launcher import Process,Channel
        with tempfile.TemporaryDirectory() as d:
            home=Path(d);entry=home/'entry';entry.mkdir()
            native=entry/'native_fixture';native.write_text((ROOT/'tests/navigation_peer.py').read_text());native.chmod(0o700)
            a,b=socket.socketpair();self.addCleanup(a.close);self.addCleanup(b.close)
            agent=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'])
            def cleanup():
                if agent.poll() is None:agent.terminate()
                agent.wait(timeout=3)
            self.addCleanup(cleanup)
            existing=Process(agent.pid,os.getuid(),tuple(agent.args))
            base=yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text())
            stopped=threading.Event();timer=threading.Timer(.45,stopped.set);timer.start();self.addCleanup(timer.cancel)
            real_popen=subprocess.Popen;real_read=Path.read_bytes
            def spawn(argv,**kwargs):return real_popen([*argv,'--test-fd='+str(b.fileno())],**kwargs,pass_fds=(b.fileno(),))
            def read_bytes(path):
                if path==Path('/proc',str(agent.pid),'cmdline'):return b'\0'.join(os.fsencode(x) for x in agent.args)+b'\0'
                return real_read(path)
            start=time.monotonic()
            with patch.object(Path,'home',return_value=home),patch.object(standby,'preflight',return_value=(native,existing)),\
                 patch.object(standby,'verify_binary'),patch.object(standby.subprocess,'Popen',side_effect=spawn),\
                 patch.object(standby,'connect_controller',return_value=Channel(a)),patch.object(Path,'read_bytes',new=read_bytes):
                standby.supervise(home/'base',home/'agent',home/'FC01_navigation',stopped,
                    controller=native,config_factory=lambda *_:m.navigation_profile(base),
                    status_metadata={'rc_navigation_enabled':True,'flight_actions_enabled':True},row_formatter=m.describe_row)
            self.assertGreater(time.monotonic()-start,.7,'native recovery must outlive supervisor stop request')
            self.assertIsNone(agent.poll())
            trace=json.loads((home/'FC01_navigation/trace.json').read_text())
            self.assertEqual(trace['ops'],['STATUS'])
            self.assertTrue(trace['rc_enabled'] and trace['automatic_arm_disabled'])
            self.assertTrue(trace['external_log_disabled'])
            last=json.loads((home/'FC01_navigation/status.json').read_text())
            self.assertEqual(last['status'],'STOPPED')
            self.assertTrue(last['rc_navigation_enabled'] and last['flight_actions_enabled'])

    # MainPID、服务状态、快照时刻和实际源年龄同时约束显示，active 不能复活旧数据。
    def test_current_service_instance_controls_snapshot_validity(self):
        self.api()
        self.assertTrue((ROOT/'navigation.py').exists(),'navigation management/view missing')
        m=importlib.import_module('navigation')
        row=dict(status='READY_WAIT_ARM',reason='ready',supervisor_pid=10,updated_unix=1000,
                 state_age_seconds=.1,feedback_current=True,
                 state=dict(phase='RC_READY_WAIT_ARM',armed=False,nav_state=15,relative_height_m=.01,
                            status_fresh=True,local_fresh=True,source_timing={
                                name:dict(source_reason='VALID',receipt_reason='VALID',timeout_s=timeout,
                                          envelope_age_s=.02,sample_age_s=.02,receipt_age_s=.02,
                                          sample_progress_age_s=.02)
                                for name,timeout in (('status',2.5),('land',2.5),('local',.5),
                                                     ('attitude',.5),('flags',2.5),('battery',1.))}))
        self.assertIn('STARTING',m.render_status(row,{'ActiveState':'active','MainPID':'11'},now=1001))
        self.assertIn('STALE',m.render_status(row,{'ActiveState':'active','MainPID':'10'},now=1020))
        self.assertIn('[READY_WAIT_ARM]',m.render_status(row,{'ActiveState':'active','MainPID':'10'},now=1000.1))
        self.assertIn('[STALE]',m.render_status(row,{'ActiveState':'active','MainPID':'10'},now=1001))
        self.assertIn('STOPPED',m.render_status(row,{'ActiveState':'inactive','MainPID':'0'},now=1001))

    # 监督器角色元数据独立保留，不用于填补原生动作门的真假。
    def test_role_metadata_does_not_override_actual_native_gate_values(self):
        self.api();import standby
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)
            standby.write_status(path,{'status':'WAITING','rc_navigation_enabled':True,'flight_actions_enabled':True})
            row=json.loads((path/'status.json').read_text())
            self.assertTrue(row['rc_navigation_enabled'] and row['flight_actions_enabled'])

    # 旧 boot_id 或无法解析的时间戳使视图降级，不允许 READY 提示。
    def test_view_rejects_previous_boot_and_malformed_time(self):
        self.api();m=importlib.import_module('navigation')
        row=dict(status='READY_WAIT_ARM',reason='ready',supervisor_pid=10,updated_unix=1000,boot_id='old')
        info={'ActiveState':'active','MainPID':'10','boot_id':'new'}
        self.assertIn('STARTING',m.render_status(row,info,now=1001))
        row['boot_id']='new';row['updated_unix']='bad timestamp'
        self.assertIn('STALE',m.render_status(row,info,now=1001))

    # 关闭状态窗口只结束显示，不能发 stop/restart 或管理后台进程。
    def test_closed_read_only_view_does_not_stop_or_manage_service(self):
        self.api();m=importlib.import_module('navigation')
        class ClosedOutput:
            def isatty(self):return False
            def write(self,_):raise BrokenPipeError('view closed')
            def flush(self):pass
        row=dict(supervisor_pid=10,status='WAITING',reason='ready',updated_unix=time.time())
        with patch.object(m,'service_info',return_value={'ActiveState':'active','MainPID':'10'}),\
             patch.object(m,'snapshot',return_value=row),\
             patch.object(m.subprocess,'run',side_effect=AssertionError('view cannot issue service operations')),\
             contextlib.redirect_stdout(ClosedOutput()):
            m.monitor(Path('/unused'))

import test_native_profile as native_contract
# 用真实 C++ 参数验证器复核 Python 生成的人工 RC 配置。
class NavigationNativeConfigContract(native_contract.NativeProfileContract):
    # 生成的 RC 人工路线须满足当前原生 Config::validate。
    def test_actual_generated_rc_configuration_is_valid(self):
        m=importlib.import_module('navigation_service')
        p=m.navigation_profile(yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text()))['fc01_system']['ros__parameters']
        result=self.run_native(p)
        self.assertEqual(result.returncode,0,result.stderr)
    # 人工 RC 路线开启程序解锁是合同冲突，原生必须拒绝。
    def test_actual_native_rejects_automatic_arm_in_manual_route(self):
        m=importlib.import_module('navigation_service')
        p=m.navigation_profile(yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text()))['fc01_system']['ros__parameters']
        p['arm_enabled']=True
        self.assertNotEqual(self.run_native(p).returncode,0)
        p['arm_enabled']=False
        for slot in (0,7):
            p['rc_navigation_mode_slot']=slot
            self.assertNotEqual(self.run_native(p).returncode,0)

    # 六个合法档位逐个通过真实原生验证，避免 Python/原生允许范围分歧。
    def test_all_valid_mode_slots_are_accepted_by_actual_native_config(self):
        m=importlib.import_module('navigation_service')
        p=m.navigation_profile(yaml.safe_load((ROOT/'tests/fixtures/base.yaml').read_text()))['fc01_system']['ros__parameters']
        for slot in range(1,7):
            with self.subTest(slot=slot):
                p['rc_navigation_mode_slot']=slot
                result=self.run_native(p)
                self.assertEqual(result.returncode,0,result.stderr)

if __name__=='__main__':unittest.main()
