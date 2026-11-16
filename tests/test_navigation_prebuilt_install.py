"""预编译安装事务离线回归；fake systemd/proc绝不操作宿主服务或设备。"""
import copy
import ast
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import types
import unittest
from unittest.mock import patch

import yaml
import navigation_prebuilt_install as m


def elf(machine=183):
    data=bytearray(64);data[:6]=b'\x7fELF\x02\x01';data[18:20]=machine.to_bytes(2,'little')
    return bytes(data)+b'FC01_TEST_ONLY_NOT_EXECUTABLE'


class Fixture:
    def __init__(self, temp):
        self.home=Path(temp);self.uid=os.getuid();self.user='orangepi'
        self.units=self.home/'units';self.units.mkdir()
        self.old=self.home/'old';self.old.mkdir()
        self.binary=self.old/'fc01_system_node';self.binary.write_bytes(elf());self.binary.chmod(0o700)
        self.runtime=self.home/'FC01_navigation/runtime.yaml';self.runtime.parent.mkdir()
        self.config={'fc01_system':{'ros__parameters':dict(fc01_runtime_mode='flight-test',
            profile='attitude_altitude',flight_enabled=False,flight_actions_enabled=True,
            takeoff_enabled=True,flight_parameters_confirmed=True,bench_arm_only_enabled=False,
            rc_takeover_confirmed=True,autonomous_failsafe_confirmed=False,
            rc_target_height=1.23,rc_navigation_mode_slot=1,rc_navigation_enabled=True,
            arm_enabled=False,visual_navigation_enabled=False,aa_hover_thrust=.5)}}
        self.runtime.write_text(yaml.safe_dump(self.config))
        self.sdk=self.home/'sdk';self.sdk.mkdir()
        self.unit=self.units/m.SERVICE
        self.unit.write_text(m.MARKER+'\n[Service]\nUser=orangepi\nWorkingDirectory='+str(self.old)+'\nExecStart=/old/navigation_boot.sh run --mode flight-test\n')
        self.original_unit=self.unit.read_bytes();self.original_runtime=self.runtime.read_bytes()
        self.info=dict(FragmentPath=str(self.unit),DropInPaths='',MainPID='10',ActiveState='active',
            UnitFileState='enabled',User=self.user,WorkingDirectory=str(self.old),
            Environment='ROS_DOMAIN_ID=0',EnvironmentFiles='',ExecStart='/old/navigation_boot.sh')
        self.old_controller=dict(pid=12,uid=self.uid,argv=[str(self.binary),'--ros-args','--params-file',str(self.runtime),'-p','socket_path:=/fake/control.sock'],exe=str(self.binary),cwd=str(self.old),sha256=m.digest(self.binary))
        self.controller=copy.deepcopy(self.old_controller)
        self.env=dict(ROS_DOMAIN_ID='0',ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',FC01_PROJECT_ROOT=str(self.sdk),FASTRTPS_DEFAULT_PROFILES_FILE='/unchanged/profile.xml')
        self.calls=[];self.fail_deps=False;self.fail_new_start=False;self.fail_new_verify=False
        self.invalid_ground=False;self.stuck=False;self.now=0
        self.agent_owned=False;self.agent_pid=55
        self.agent=dict(pid=55,uid=self.uid,exe='/fake/MicroXRCEAgent',sha256='agent-sha',
            argv=['/fake/MicroXRCEAgent','serial','-D','/dev/ttyS2','-b','115200','-v','0'])

    def backend(self):
        return self

    def check_host(self):self.calls.append('check_host')
    def authenticate(self):self.calls.append('authenticate')
    def service(self):return dict(self.info)
    def controllers(self):return [copy.deepcopy(self.controller)] if self.controller else []
    def descendant(self,pid,main):return main in (10,20) and (pid in (12,22) or (self.agent_owned and self.agent and pid==self.agent['pid']))
    def environment(self,pid):return dict(self.env)
    def retained_agent(self,expected=None,*,allow_missing=False):
        if self.agent is None:
            if allow_missing:return None
            raise m.InstallError('Agent absent')
        row=copy.deepcopy(self.agent)
        if expected is not None and row!=expected:raise m.InstallError('Agent changed')
        return row
    def boot_id(self):return 'test-boot'
    def live_state(self,controller):
        config=yaml.safe_load(Path(controller['argv'][controller['argv'].index('--params-file')+1]).read_text())
        p=config['fc01_system']['ros__parameters']
        result=dict(controller_pid=controller['pid'],armed=self.invalid_ground,
            active=False,nav_state=2,heartbeat_enabled=False,
            arming_state_known=True,status_fresh=True,land_fresh=True,local_fresh=True,
            attitude_fresh=True,source_clock_valid=True,landed=True,at_rest=True,thrust_body=[0,0,0],
            rc_navigation_enabled=True,rc_axes_safe=True,rc_arm_switch=3,rc_kill_switch=3,
            rc_mode_slot=6,rc_navigation_mode_slot=1,rc_source_valid=True,rc_switches_fresh=True,
            rc_link_ready=True,rc_switches_ready=True,rc_source_clock_valid=True,rc_switch_clock_valid=True,
            rc_target_height=p['rc_target_height'])
        for key in ('profile','flight_enabled','flight_actions_enabled','arm_enabled','takeoff_enabled',
                    'flight_parameters_confirmed','bench_arm_only_enabled','visual_navigation_enabled',
                    'rc_takeover_confirmed','autonomous_failsafe_confirmed'):
            result[key]=p[key]
        return result
    def dependencies(self,entry,manifest,env):
        self.calls.append('dependencies')
        if self.fail_deps:raise m.InstallError('missing library')
    def config_check(self,*args):self.calls.append('config_check')
    def systemd_verify(self,*args):self.calls.append('systemd_verify')
    def write_root(self,path,data,mode=0o644):
        self.calls.append('write_root');path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data);path.chmod(mode)
    def remove_root(self,path):
        self.calls.append('remove_root');Path(path).unlink(missing_ok=True)
    def reload(self):
        self.calls.append('reload')
        own=self.units/(m.SERVICE+'.d')/m.DROPIN
        if own.is_file():
            line=next(l for l in own.read_text().splitlines() if l.startswith('ExecStart=/bin/bash '))
            entry=Path(line.split('"')[1]).parent
            self.info.update(WorkingDirectory=str(entry),ExecStart='/bin/bash '+str(entry/'prebuilt_boot.sh'),DropInPaths=str(own))
        else:self.info.update(WorkingDirectory=str(self.old),ExecStart='/old/navigation_boot.sh',DropInPaths='')
    def stop(self):
        self.calls.append('stop')
        if not self.stuck:
            self.controller=None;self.info.update(MainPID='0',ActiveState='inactive')
            if self.agent_owned:self.agent=None
    def start(self):
        self.calls.append('start')
        entry=Path(self.info['WorkingDirectory'])
        if self.agent is None:
            self.agent_pid+=1
            self.agent=dict(pid=self.agent_pid,uid=self.uid,exe='/fake/MicroXRCEAgent',sha256='agent-sha',
                argv=['/fake/MicroXRCEAgent','serial','-D','/dev/ttyS2','-b','115200','-v','0'])
        if entry==self.old:
            self.controller=copy.deepcopy(self.old_controller);self.info.update(MainPID='10',ActiveState='active')
        else:
            if self.fail_new_start:raise m.InstallError('new start failed')
            manifest=json.loads((entry/m.MANIFEST).read_text());binary=entry/manifest['controller_relative']
            self.runtime.write_bytes((entry/'deployment/runtime.yaml').read_bytes())
            self.controller=dict(pid=22,uid=self.uid,argv=[str(binary),'--ros-args','--params-file',str(self.runtime),'-p','socket_path:=/fake/new.sock'],exe=str(binary),cwd=str(entry),sha256=m.digest(binary))
            self.info.update(MainPID='20',ActiveState='active')
    def sleep(self,s):
        if self.fail_new_verify:raise m.InstallError('new verify failed')
        if self.stuck:raise m.InstallError('old controller still running')

    def archive(self,machine=183,extra=None):
        content={'install/lib/jikl_fc01_system/fc01_system_node':elf(machine),
            'install/lib/jikl_fc01_system/fc01_config_check':elf(machine),
            'prebuilt_boot.sh':b'#!/bin/bash\n', 'navigation_prebuilt_service.py':b'# entry\n'}
        manifest=dict(version='20261116.stage3.test',source_commit='a'*40,architecture='aarch64',
            ubuntu='22.04',ros_distro='humble',controller_relative='install/lib/jikl_fc01_system/fc01_system_node',
            payload_sha256={name:hashlib.sha256(data).hexdigest() for name,data in content.items()})
        content[m.MANIFEST]=json.dumps(manifest).encode()
        archive=self.home/('runtime'+str(machine)+'.tar.gz')
        with tarfile.open(archive,'w:gz') as tar:
            for name,data in content.items():
                item=tarfile.TarInfo('FC01_OrangePi_20261116_stage3_arm64/'+name);item.size=len(data);item.mode=0o700
                tar.addfile(item,io.BytesIO(data))
            if extra:tar.addfile(extra)
        return archive,m.digest(archive)


class InstallerTest(unittest.TestCase):
    @staticmethod
    def timed_state(stamp):
        fields=dict(timeout_s=.5,source_reason='VALID',receipt_reason='VALID',
                    envelope_age_s=.01,receipt_age_s=.01,sample_age_s=.01,sample_progress_age_s=.01)
        return dict(controller_pid=12,diagnostic_monotonic_s=stamp,
                    source_timing={name:dict(fields) for name in
                        ('status','land','local','attitude','flags','battery')})

    def test_install_repeat_rollback_and_effective_inheritance(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive()
            result=m.install(archive,sha,f)
            self.assertFalse(result['already_active']);self.assertEqual(f.calls.count('stop'),1)
            entry=Path(result['entry']);config=yaml.safe_load((entry/'deployment/runtime.yaml').read_text())
            new=config['fc01_system']['ros__parameters'];old=f.config['fc01_system']['ros__parameters']
            for key,value in old.items():self.assertEqual(new[key],value)
            self.assertEqual(new['fc01_config_path'],str(f.home/'FC01_navigation/runtime.yaml'))
            inherited=json.loads((entry/'deployment/environment.json').read_text())
            self.assertEqual(inherited['environment']['FASTRTPS_DEFAULT_PROFILES_FILE'],'/unchanged/profile.xml')
            self.assertEqual(f.unit.read_bytes(),f.original_unit)
            dropin=(f.units/(m.SERVICE+'.d')/m.DROPIN).read_text()
            self.assertIn('\nWorkingDirectory='+str(entry)+'\n',dropin)
            self.assertNotIn('WorkingDirectory="',dropin)
            again=m.install(archive,sha,f)
            self.assertTrue(again['already_active']);self.assertEqual(f.calls.count('stop'),1)
            self.assertEqual(again['backup'],result['backup'])
            m.rollback(result['backup'],f)
            self.assertEqual(f.unit.read_bytes(),f.original_unit);self.assertEqual(f.runtime.read_bytes(),f.original_runtime)
            self.assertEqual(f.controller['exe'],str(f.binary));self.assertEqual(f.info['ActiveState'],'active')
            self.assertFalse((f.units/(m.SERVICE+'.d')/m.DROPIN).exists())

    @unittest.skipUnless(shutil.which('systemd-analyze'),'requires real systemd parser')
    def test_real_systemd_parser_accepts_scalar_working_directory(self):
        # 使用宿主真实解析器，不启动unit、不调用systemctl、不要求root。
        with tempfile.TemporaryDirectory(prefix='fc01 unit % ') as temp:
            root=Path(temp);boot=root/'prebuilt_boot.sh';boot.write_text('#!/bin/bash\n')
            unit=root/'fc01-scalar-test.service'
            unit.write_text('[Unit]\nDescription=FC01 parser-only fixture\n[Service]\nType=simple\n'
                +'ExecStart=/bin/bash '+m.systemd_quote(boot)+'\n'
                +'WorkingDirectory='+m.systemd_directory(root)+'\n')
            result=subprocess.run(['systemd-analyze','verify',str(unit)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertNotIn('WorkingDirectory="',unit.read_text())
            self.assertIn('%%',unit.read_text())

    def test_scalar_working_directory_rejects_ambiguous_values(self):
        for value in ('relative','/tmp/new\n[Service]','/tmp/new\\path','/tmp/new\tpath','/tmp/new '):
            with self.subTest(value=value),self.assertRaises(m.InstallError):m.systemd_directory(value)

    def test_bad_hash_and_architecture_rejected_before_stop(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'SHA-256'):m.install(archive,'0'*64,f)
            archive,sha=f.archive(machine=62)
            with self.assertRaisesRegex(m.InstallError,'AArch64'):m.install(archive,sha,f)
            self.assertNotIn('stop',f.calls);self.assertEqual(f.runtime.read_bytes(),f.original_runtime)

    def test_missing_dependency_rejected_before_stop(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.fail_deps=True;archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'missing library'):m.install(archive,sha,f)
            self.assertNotIn('stop',f.calls);self.assertNotIn('write_root',f.calls)

    def test_unknown_or_armed_controller_rejected_without_changes(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.invalid_ground=True;archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'未解锁'):m.install(archive,sha,f)
            self.assertNotIn('stop',f.calls)

    def test_unsupported_domain_preserved_and_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.env['ROS_DOMAIN_ID']='9';archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'非域0'):m.install(archive,sha,f)
            self.assertEqual(f.env['ROS_DOMAIN_ID'],'9');self.assertNotIn('stop',f.calls)

    def test_archive_traversal_and_symlink_rejected(self):
        for name,typecode in [('FC01_OrangePi_20261116_stage3_arm64/../../outside',tarfile.REGTYPE),('FC01_OrangePi_20261116_stage3_arm64/link',tarfile.SYMTYPE)]:
            with self.subTest(name=name),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);item=tarfile.TarInfo(name);item.type=typecode;item.linkname='/etc/passwd'
                archive,sha=f.archive(extra=item)
                with self.assertRaisesRegex(m.InstallError,'不安全'):m.install(archive,sha,f)
                self.assertNotIn('stop',f.calls)

    def test_failed_new_start_restores_old_active(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.fail_new_start=True;archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'原版配置和运行状态已恢复'):m.install(archive,sha,f)
            self.assertEqual(f.controller['exe'],str(f.binary));self.assertEqual(f.info['ActiveState'],'active')
            self.assertEqual(f.runtime.read_bytes(),f.original_runtime)

    def test_original_dropin_and_environment_file_backed_up_unchanged(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);drop=f.units/(m.SERVICE+'.d')/'10-original.conf';drop.parent.mkdir();drop.write_text('[Service]\nEnvironment=ROS_DOMAIN_ID=0\n')
            envfile=f.home/'dds.env';envfile.write_text('RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n')
            f.info.update(DropInPaths=str(drop),EnvironmentFiles=str(envfile)+' (ignore_errors=no)')
            archive,sha=f.archive();r=m.install(archive,sha,f)
            backup=Path(r['backup']);saved=json.loads((backup/'BACKUP.json').read_text())
            self.assertIn(str(drop),[x['path'] for x in saved['files']]);self.assertIn(str(envfile),[x['path'] for x in saved['files']])
            self.assertEqual(drop.read_text(),'[Service]\nEnvironment=ROS_DOMAIN_ID=0\n')

    def test_corrupted_existing_payload_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            binary=Path(result['entry'])/'install/lib/jikl_fc01_system/fc01_system_node'
            binary.write_bytes(elf()+b'edited')
            before=f.calls.count('stop')
            with self.assertRaisesRegex(m.InstallError,'摘要'):m.install(archive,sha,f)
            self.assertEqual(f.calls.count('stop'),before)
            self.assertTrue(binary.read_bytes().endswith(b'edited'))

    def test_busy_old_controller_never_changes_service(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.stuck=True;archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'原版配置和运行状态已恢复'):m.install(archive,sha,f)
            self.assertEqual(f.unit.read_bytes(),f.original_unit)
            self.assertEqual(f.controller['exe'],str(f.binary))
            self.assertNotIn('write_root',f.calls)

    def test_duplicate_controller_rejects_without_stop(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive()
            f.controllers=lambda:[copy.deepcopy(f.controller),copy.deepcopy(f.controller)]
            with self.assertRaisesRegex(m.InstallError,'恰好一个'):m.install(archive,sha,f)
            self.assertNotIn('stop',f.calls)

    def test_nan_or_incomplete_zero_thrust_proof_rejects(self):
        for bad in ([],[0,0,float('nan')],[0,0,1.e-10]):
            with self.subTest(bad=bad),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);archive,sha=f.archive();original=f.live_state
                def invalid(controller):
                    result=original(controller);result['thrust_body']=bad;return result
                f.live_state=invalid
                with self.assertRaisesRegex(m.InstallError,'零推力'):m.install(archive,sha,f)
                self.assertNotIn('stop',f.calls)

    def test_ground_but_active_or_stale_rc_never_authorizes_switch(self):
        for key,value in (('active',True),('nav_state',14),('heartbeat_enabled',True),
                          ('rc_source_clock_valid',False),('rc_mode_slot',0)):
            with self.subTest(key=key),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);archive,sha=f.archive();state=f.live_state
                def invalid(controller):
                    row=state(controller);row[key]=value;return row
                f.live_state=invalid
                with self.assertRaises(m.InstallError):m.install(archive,sha,f)
                self.assertNotIn('stop',f.calls)

    def test_ground_ready_prestream_does_not_require_heartbeat_off(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);state=f.live_state
            def ready(controller):
                row=state(controller)
                row.update(active=True,heartbeat_enabled=True,phase='RC_READY_WAIT_ARM',nav_state=15)
                return row
            f.live_state=ready
            self.assertEqual(m.require_ground(f)[0]['pid'],12)

    def test_old_or_pid_reused_snapshot_never_gets_freshened(self):
        state=self.timed_state(100.)
        m.validate_snapshot_age(state,100.1,90.)
        with self.assertRaisesRegex(m.InstallError,'已旧'):m.validate_snapshot_age(state,101.,90.)
        with self.assertRaisesRegex(m.InstallError,'本次进程'):m.validate_snapshot_age(state,100.1,100.01)
        state['source_timing']['local']['sample_age_s']=.45
        with self.assertRaisesRegex(m.InstallError,'过期'):m.validate_snapshot_age(state,100.1,90.)

    def test_supervisor_snapshot_uses_identity_not_a_second_ipc_client(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.proc=f.home/'proc';(f.proc/'12').mkdir(parents=True)
            # stat fields: after comm, field3 is index0 and field22 starttime is index19.
            fields=['S']+['0']*18+[str(int(90*os.sysconf('SC_CLK_TCK')))]
            (f.proc/'12/stat').write_text('12 (fc01_system_node) '+' '.join(fields))
            row=dict(supervisor_pid=10,boot_id='test-boot',controller_pid=12,state_controller_pid=12,
                     status='READY_WAIT_ARM',feedback_current=True,state=self.timed_state(100.))
            path=f.home/'FC01_navigation/status.json';path.write_text(json.dumps(row))
            with patch.object(m.time,'monotonic',return_value=100.1):
                state=m.Backend.live_state(f,f.controller)
            self.assertEqual(state['diagnostic_monotonic_s'],100.)
            row['boot_id']='old-boot';path.write_text(json.dumps(row))
            f.sleep=lambda _: (_ for _ in ()).throw(m.InstallError('refused stale snapshot'))
            with patch.object(m.time,'monotonic',return_value=100.1):
                with self.assertRaisesRegex(m.InstallError,'refused stale'):m.Backend.live_state(f,f.controller)

    def test_slow_service_query_precedes_latest_snapshot_read_without_extending_ttl(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.proc=f.home/'proc';(f.proc/'12').mkdir(parents=True)
            fields=['S']+['0']*18+[str(int(90*os.sysconf('SC_CLK_TCK')))]
            (f.proc/'12/stat').write_text('12 (fc01_system_node) '+' '.join(fields))
            path=f.home/'FC01_navigation/status.json'
            row=dict(supervisor_pid=10,boot_id='test-boot',controller_pid=12,state_controller_pid=12,
                     status='READY_WAIT_ARM',feedback_current=True,state=self.timed_state(99.))
            path.write_text(json.dumps(row))
            calls=[];service=f.service;clock=[100.]
            def slow_query():
                calls.append('service');clock[0]+=.13
                # 查询耗时期间监督器发布新原生事实：必须在查询之后读取它。
                row['state']=self.timed_state(clock[0]-.02)
                path.write_text(json.dumps(row))
                return service()
            f.service=slow_query;read=Path.read_text
            def read_trace(p,*args,**kwargs):
                if p==path:calls.append('snapshot')
                return read(p,*args,**kwargs)
            with patch.object(m.time,'monotonic',side_effect=lambda:clock[0]),patch.object(Path,'read_text',read_trace):
                state=m.Backend.live_state(f,f.controller)
            self.assertEqual(calls[:2],['service','snapshot'])
            self.assertAlmostEqual(state['diagnostic_monotonic_s'],100.11)
            # 相同窗口不变：若没有新原生事实，不能把旧99s状态改写为当前时间。
            f.service=service;row['state']=self.timed_state(99.);path.write_text(json.dumps(row))
            f.sleep=lambda _: (_ for _ in ()).throw(m.InstallError('stale remains stale'))
            with patch.object(m.time,'monotonic',return_value=100.13):
                with self.assertRaisesRegex(m.InstallError,'stale remains stale'):m.Backend.live_state(f,f.controller)

    def test_unknown_supervisor_owned_agent_arguments_refuse_before_stop(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.agent_owned=True;f.agent['argv']+=['--unknown']
            archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'Agent参数'):m.install(archive,sha,f)
            self.assertNotIn('stop',f.calls)

    def test_owned_agent_handoff_install_repeat_rollback_preserves_exe_arguments(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.agent_owned=True;original=copy.deepcopy(f.agent);archive,sha=f.archive()
            result=m.install(archive,sha,f)
            self.assertNotEqual(f.agent['pid'],original['pid'])
            for key in ('exe','sha256','argv'):self.assertEqual(f.agent[key],original[key])
            pid=f.agent['pid'];self.assertTrue(m.install(archive,sha,f)['already_active'])
            self.assertEqual(f.agent['pid'],pid)
            m.rollback(result['backup'],f)
            self.assertNotEqual(f.agent['pid'],pid)
            for key in ('exe','sha256','argv'):self.assertEqual(f.agent[key],original[key])
            self.assertEqual(f.controller['exe'],str(f.binary))

    def test_shared_agent_disappearance_is_not_treated_as_owned_handoff(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();stop=f.stop
            def lose_shared():stop();f.agent=None
            f.stop=lose_shared
            f.sleep=lambda _: (_ for _ in ()).throw(m.InstallError('shared Agent not restored'))
            with self.assertRaises(m.InstallError):m.install(archive,sha,f)
            self.assertNotIn('write_root',f.calls)

    def test_owned_agent_new_start_failure_restores_original_with_same_parameters(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.agent_owned=True;f.fail_new_start=True;original=copy.deepcopy(f.agent)
            archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'原版配置和运行状态已恢复'):m.install(archive,sha,f)
            for key in ('exe','sha256','argv'):self.assertEqual(f.agent[key],original[key])
            self.assertEqual(f.controller['exe'],str(f.binary))

    def test_owned_agent_handoff_changed_program_is_never_accepted(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.agent_owned=True;original=copy.deepcopy(f.agent)
            f.agent['sha256']='different-binary'
            with self.assertRaisesRegex(m.InstallError,'摘要'):m.verify_agent(f,original,True)

    def test_real_backend_missing_agent_requires_clean_free_uart_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            backend=object.__new__(m.Backend);backend.proc=Path(temp);backend.uid=os.getuid()
            clean=types.SimpleNamespace(returncode=1,stdout='',stderr='')
            denied=types.SimpleNamespace(returncode=1,stdout='',stderr='Permission denied')
            with patch.object(Path,'is_char_device',return_value=True):
                with patch.object(m.subprocess,'run',return_value=clean):
                    self.assertIsNone(backend.retained_agent(allow_missing=True))
                with patch.object(m.subprocess,'run',return_value=denied):
                    with self.assertRaises(m.InstallError):backend.retained_agent(allow_missing=True)
            with patch.object(Path,'is_char_device',return_value=False):
                with patch.object(m.subprocess,'run',return_value=clean):
                    with self.assertRaises(m.InstallError):backend.retained_agent(allow_missing=True)

    def test_real_size_manifest_never_enters_dependency_process_argv(self):
        # 真实发行约5000成员；单个Linux argv不能超过MAX_ARG_STRLEN约128KiB。
        # 检查真正Backend生成的命令及内层Python，而不是Fixture的空dependencies替身。
        manifest={'payload_sha256':{
            'install/lib/project/'+('long_source_name_'*4)+str(i):'a'*64 for i in range(5000)}}
        self.assertGreater(len(json.dumps(manifest)),131072)
        commands=[]
        backend=object.__new__(m.Backend)
        def capture(argv):
            commands.append(argv);return types.SimpleNamespace(stdout='dependencies checked')
        backend.run=capture
        result=backend.dependencies(Path('/independent/release'),manifest,{})
        self.assertEqual(result,'dependencies checked')
        command=commands[0]
        self.assertTrue(all(len(arg.encode())<131072 for arg in command))
        self.assertEqual(command[-1],'/independent/release')
        self.assertEqual(len(command),5)
        script=command[2]
        self.assertIn('exec /usr/bin/python3 - "$1" <<',script)
        nested=script.split("<<'CHECK'\n",1)[1].rsplit('\nCHECK',1)[0]
        tree=ast.parse(nested)
        indices=[node.slice.value for node in ast.walk(tree)
                 if isinstance(node,ast.Subscript) and isinstance(node.value,ast.Attribute)
                 and isinstance(node.value.value,ast.Name) and node.value.value.id=='sys'
                 and node.value.attr=='argv' and isinstance(node.slice,ast.Constant)]
        self.assertEqual(indices,[1])

    def test_startup_acceptance_waits_for_measured_slow_preflight_not_stale_feedback(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            entry=Path(result['entry']);manifest=json.loads((entry/m.MANIFEST).read_text())
            environment=json.loads((entry/'deployment/environment.json').read_text())
            controller=f.controllers;clock=[0.]
            f.controllers=lambda:[] if clock[0]<40 else controller()
            f.sleep=lambda seconds:clock.__setitem__(0,clock[0]+10)
            with patch.object(m.time,'monotonic',side_effect=lambda:clock[0]):
                actual=m.verify_active(f,entry,manifest,environment)
            self.assertGreaterEqual(clock[0],40)
            self.assertEqual(actual['controller']['exe'],str(entry/manifest['controller_relative']))
            self.assertEqual(m.STARTUP_ACCEPTANCE_SECONDS,180)
            # 同一验收等待仍有界；永远没控制器必须失败，不能变成无限重试。
            f.controllers=lambda:[];clock[0]=0
            with patch.object(m.time,'monotonic',side_effect=lambda:clock[0]):
                with self.assertRaisesRegex(m.InstallError,'启动验收失败'):
                    m.verify_active(f,entry,manifest,environment)
            self.assertEqual(clock[0],180)

    def test_live_rc_configuration_mismatch_never_silently_inherited(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();state=f.live_state
            def mismatch(controller):
                row=state(controller);row['rc_navigation_mode_slot']=3;return row
            f.live_state=mismatch
            with self.assertRaisesRegex(m.InstallError,'RC/动作门'):m.install(archive,sha,f)
            self.assertNotIn('stop',f.calls)

    def test_repeat_install_without_legacy_sdk_environment(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            f.env.pop('FC01_PROJECT_ROOT')
            self.assertTrue(m.install(archive,sha,f)['already_active'])
            self.assertEqual(f.calls.count('stop'),1)

    def test_inactive_failed_candidate_can_roll_back_without_live_controller(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            f.controller=None;f.info.update(ActiveState='failed',MainPID='0')
            m.rollback(result['backup'],f)
            self.assertEqual(f.controller['exe'],str(f.binary))

    def test_original_restart_different_parameters_is_not_reported_recovered(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            backup=Path(result['backup']);record=json.loads((backup/'BACKUP.json').read_text())
            f.controller=copy.deepcopy(f.old_controller)
            f.info.update(ActiveState='active',MainPID='10')
            wrong=copy.deepcopy(f.config);wrong['fc01_system']['ros__parameters']['rc_target_height']=2.
            f.runtime.write_text(yaml.safe_dump(wrong))
            with self.assertRaisesRegex(m.InstallError,'参数.*原始备份'):
                m.verify_original(f,backup,record,timeout=0)

    def test_external_unit_edit_blocks_rollback_before_overwrite(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            f.unit.write_bytes(f.original_unit+b'# operator edit\n')
            before=f.calls.count('stop')
            with self.assertRaisesRegex(m.InstallError,'外部修改'):m.rollback(result['backup'],f)
            self.assertTrue(f.unit.read_bytes().endswith(b'# operator edit\n'))
            self.assertEqual(f.calls.count('stop'),before)
            self.assertEqual(f.info['ActiveState'],'active')

    def test_invalid_backup_rejected_without_configuration_write(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            (Path(result['backup'])/'runtime.yaml').write_text('corrupt')
            count=f.calls.count('write_root')
            before=f.calls.count('stop')
            with self.assertRaisesRegex(m.InstallError,'备份摘要'):m.rollback(result['backup'],f)
            self.assertEqual(f.calls.count('write_root'),count)
            self.assertEqual(f.calls.count('stop'),before)
            self.assertEqual(f.info['ActiveState'],'active')

    def test_failed_supervisor_without_controller_restores_old_active(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();start=f.start
            def start_without_new_controller():
                if Path(f.info['WorkingDirectory'])==f.old:return start()
                f.calls.append('start');f.controller=None
                f.info.update(ActiveState='active',MainPID='20')
            f.start=start_without_new_controller;f.fail_new_verify=True
            with self.assertRaisesRegex(m.InstallError,'原版配置和运行状态已恢复'):
                m.install(archive,sha,f)
            self.assertEqual(f.info['ActiveState'],'active')
            self.assertEqual(f.controller['exe'],str(f.binary))
            self.assertEqual(f.calls.count('stop'),2)

    def test_unmodified_sources_are_never_privileged_rewritten_on_rollback(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            calls=[];write=f.write_root
            def record_write(path,data,mode=0o644):
                calls.append(Path(path));return write(path,data,mode)
            f.write_root=record_write
            m.rollback(result['backup'],f)
            self.assertNotIn(f.unit,calls)
            self.assertEqual(f.unit.read_bytes(),f.original_unit)


if __name__=='__main__':unittest.main()
