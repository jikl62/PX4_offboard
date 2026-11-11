import configparser
import importlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]

class ProbeTests(unittest.TestCase):
    def evidence(self):
        return importlib.import_module('navigation_probe').GroundEvidence()
    def observe(self,e,kind,stamp,**extra):
        msg=dict(timestamp=int(stamp*1e6),system_id=1,component_id=1,arming_state=1,landed=True,at_rest=True)
        msg.update(extra);e.observe(kind,msg,stamp,stamp-100)
    def test_probe_requires_advancing_clock_and_exclusive_fresh_ground(self):
        e=self.evidence()
        for stamp in (1000.,1001.):
            self.observe(e,'status',stamp);self.observe(e,'land',stamp)
        self.assertTrue(e.ready(1001.,901.,1,1),e.reason)
        self.assertFalse(e.ready(1001.,901.,2,1))
        self.assertFalse(e.ready(1005.,905.,1,1))
    def test_duplicates_unsynchronized_and_unrest_are_not_ground_proof(self):
        for mode in ('duplicate','clock','rest','source'):
            e=self.evidence()
            for stamp in (1000.,1001.):
                t=1000. if mode=='duplicate' else stamp
                kw={'timestamp':1} if mode=='clock' else {'at_rest':False} if mode=='rest' else {}
                self.observe(e,'land',t,**kw)
                self.observe(e,'status',t,**({'system_id':2} if mode=='source' else {}))
            self.assertFalse(e.ready(1001.,901.,1,1),mode)
    def test_armed_observation_latches_even_with_invalid_clock(self):
        e=self.evidence();self.observe(e,'status',1000.,arming_state=2,timestamp=1)
        for stamp in (1001.,1002.):
            self.observe(e,'status',stamp);self.observe(e,'land',stamp)
        self.assertFalse(e.ready(1002.,902.,1,1));self.assertTrue(e.armed_seen)
    def test_live_probe_is_subscription_only_and_retains_matching_agent(self):
        import sys
        import types
        m=importlib.import_module('navigation_probe')
        rclpy=types.ModuleType('rclpy');qos=types.ModuleType('rclpy.qos');qos.qos_profile_sensor_data=object()
        msgs=types.ModuleType('px4_msgs.msg');msgs.VehicleStatus=object;msgs.VehicleLandDetected=object
        callbacks={};clock=[1000.]
        class Node:
            def create_subscription(self,cls,topic,callback,q):callbacks[topic]=callback;return object()
            def count_publishers(self,topic):return 1
            def destroy_node(self):pass
        def spin(node,timeout_sec):
            clock[0]+=.5
            for topic,cb in callbacks.items():
                cb(types.SimpleNamespace(timestamp=int(clock[0]*1e6),system_id=1,component_id=1,arming_state=1,landed=True,at_rest=True))
        rclpy.init=lambda:None;rclpy.create_node=lambda _:Node();rclpy.spin_once=spin
        rclpy.ok=lambda:True;rclpy.shutdown=lambda:None
        with tempfile.TemporaryDirectory() as d,patch.dict(sys.modules,{'rclpy':rclpy,'rclpy.qos':qos,'px4_msgs.msg':msgs}),\
             patch.object(m,'process_list',return_value=[]),patch.object(m,'uart_owners',return_value={3215}),\
             patch.object(m,'choose_agent',return_value=types.SimpleNamespace(pid=3215)),\
             patch.object(m.subprocess,'Popen',side_effect=AssertionError('existing agent must be retained')),\
             patch.object(m.time,'time',side_effect=lambda:clock[0]),patch.object(m.time,'monotonic',side_effect=lambda:clock[0]-100):
            path=Path(d)/'probe.json';self.assertEqual(m.probe(Path(d),Path(d)/'agent',path),0)
            self.assertTrue(json.loads(path.read_text())['ok'])
            self.assertEqual(len(callbacks),2)

class InstallTests(unittest.TestCase):
    def module(self):return importlib.import_module('navigation_install')
    def test_units_and_desktop_paths_support_space_dollar_percent_without_shell(self):
        m=self.module();entry=Path('/home/orangepi/Navigate 10% $x')
        text=m.render_unit('orangepi',Path('/home/orangepi'),entry,Path('/tmp/native'))
        p=configparser.ConfigParser(interpolation=None,strict=False);p.read_string(text)
        self.assertEqual(p['Service']['KillMode'],'process')
        self.assertEqual(p['Service']['TimeoutStopSec'],'infinity')
        self.assertEqual(p['Service']['SendSIGKILL'],'no')
        self.assertEqual(p['Service']['Restart'],'on-failure')
        self.assertEqual(p['Service']['RestartSec'],'30s')
        self.assertEqual(p['Unit']['StartLimitIntervalSec'],'120s')
        self.assertEqual(p['Unit']['StartLimitBurst'],'5')
        self.assertIn('10%% $$x',p['Service']['ExecStart'])
        with tempfile.TemporaryDirectory() as d:
            unit=Path(d)/m.SERVICE;unit.write_text(text)
            r=subprocess.run(['/usr/bin/systemd-analyze','verify',str(unit)],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr)
        desktop=m.render_desktop(entry)
        self.assertIn('Terminal=true',desktop);self.assertIn(' monitor',desktop)
        self.assertIn('10%%',desktop);self.assertNotIn('takeoff',desktop)
        with self.assertRaises(ValueError):m.render_desktop(Path('/tmp/bad\nExec=x'))

    def test_service_defaults_to_diagnostic_and_flight_test_is_explicit(self):
        m=self.module();entry=Path('/opt/fc01-nav');home=Path('/home/orangepi')
        diagnostic=m.render_unit('orangepi',home,entry,Path('/tmp/native'))
        self.assertIn('run --mode diagnostic',diagnostic)
        self.assertNotIn('--confirm-flight-test',diagnostic)
        with self.assertRaises(ValueError):m.render_unit('orangepi',home,entry,Path('/tmp/native'),'flight-test')
        flight=m.render_unit('orangepi',home,entry,Path('/tmp/native'),'flight-test',True)
        self.assertIn('run --mode flight-test --confirm-flight-test',flight)
    def test_build_is_offline_single_job_and_staged(self):
        m=self.module();commands=[]
        with patch.object(m.shutil,'which',return_value='/usr/bin/tool'):
            m.build_native(ROOT,Path('/tmp/staged/install'),Path('/tmp/staged/build'),
                           run=lambda argv,**kw:commands.append(argv))
        self.assertEqual(len(commands),3)
        self.assertIn('-DCMAKE_INSTALL_PREFIX=/tmp/staged/install',commands[0])
        self.assertEqual(commands[1][-2:],['--parallel','1'])
        self.assertNotIn('colcon',str(commands));self.assertNotIn('apt',str(commands))
        commands.clear()
        with patch.object(m.shutil,'which',return_value=None),self.assertRaises(RuntimeError):
            m.build_native(ROOT,Path('/tmp/a'),Path('/tmp/b'),run=lambda *a,**k:commands.append(a))
        self.assertEqual(commands,[])
    def test_ground_failure_happens_after_build_validation_before_service_mutation(self):
        m=self.module();trace=[]
        with tempfile.TemporaryDirectory() as d:
            home=Path(d);entry=home/'entry';entry.mkdir()
            def boot(_entry,_home,_project,action,*args,**kw):
                trace.append(action)
                if action=='probe':raise RuntimeError('armed')
            with patch.object(m,'boot',side_effect=boot),patch.object(m,'source_key',return_value='abc'),patch.object(m,'deploy') as deploy:
                with self.assertRaisesRegex(RuntimeError,'armed'):m.prepare(entry,home,home/'base')
                deploy.assert_not_called()
        self.assertEqual(trace,['prepare-config','build','check-config','probe'])
    def fixture(self,folder):
        m=self.module();home=Path(folder);entry=home/'entry 10% $x';entry.mkdir()
        (home/'units').mkdir();prefix=entry/'.build/abc/install';prefix.mkdir(parents=True)
        old=home/'units/fc01-standby.service'
        old.write_text(importlib.import_module('autostart').render_unit('orangepi',home,home/'old',home/'base'))
        calls=[]
        def run(argv,**kw):
            calls.append(argv)
            if 'show' in argv:
                name=argv[argv.index('show')+1];p=home/'units'/name
                return subprocess.CompletedProcess(argv,0,f'FragmentPath={p if p.exists() else ""}\nDropInPaths=\nMainPID=0\nActiveState=inactive\nUnitFileState=enabled\n','')
            if '/usr/bin/install' in argv:shutil.copyfile(argv[-2],argv[-1])
            return subprocess.CompletedProcess(argv,0,'','')
        def observe(_entry,_home,_project,action,*args):
            self.assertEqual(action,'probe');calls.append(['PROBE'])
            folder=home/'FC01_navigation';folder.mkdir(mode=0o700,exist_ok=True)
            (folder/'probe.json').write_text(json.dumps(dict(ok=True,armed_seen=False,updated_monotonic=time.monotonic(),
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())))
        patcher=patch.object(m,'boot',side_effect=observe);patcher.start();self.addCleanup(patcher.stop)
        return m,home,entry,prefix,calls,run
    def test_managed_upgrade_and_repeat_preserve_existing_candidate(self):
        with tempfile.TemporaryDirectory() as d:
            m,home,entry,prefix,calls,run=self.fixture(d)
            m.deploy(entry,home,home/'base',prefix,user='orangepi',units=home/'units',run=run,processes=[])
            self.assertEqual((entry/'install').resolve(),prefix)
            flat=[' '.join(x) for x in calls]
            self.assertTrue(any('disable --now fc01-standby.service' in x for x in flat))
            self.assertTrue(any('enable --now fc01-navigation.service' in x for x in flat))
            self.assertTrue((home/'old').exists() is False) # no old payload writes
            self.assertTrue((home/'Desktop/FC01_Navigation_Status.desktop').exists())
            calls.clear()
            m.deploy(entry,home,home/'base',prefix,user='orangepi',units=home/'units',run=run,processes=[])
            self.assertFalse(any(' stop fc01-navigation' in ' '.join(x) or 'restart' in x for x in calls))
    def test_foreign_unit_dropin_or_foreign_desktop_refuses_before_mutation(self):
        for mode in ('unit','dropin','desktop'):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as d:
                m,home,entry,prefix,calls,run=self.fixture(d)
                if mode=='unit':(home/'units/fc01-navigation.service').write_text('[Service]\nExecStart=/bin/true\n')
                if mode=='desktop':
                    (home/'Desktop').mkdir();(home/'Desktop/FC01_Navigation_Status.desktop').write_text('foreign')
                def runner(argv,**kw):
                    r=run(argv,**kw)
                    if mode=='dropin' and 'show' in argv:r.stdout+='DropInPaths=/etc/foreign.conf\n'
                    return r
                with self.assertRaises(RuntimeError):
                    m.deploy(entry,home,home/'base',prefix,user='orangepi',units=home/'units',run=runner,processes=[])
                self.assertFalse(any(x[0]=='sudo' for x in calls))
    def test_rollback_checks_ground_before_stop_and_restores_only_managed_standby(self):
        m=self.module()
        with tempfile.TemporaryDirectory() as d:
            home=Path(d);trace=[]
            with patch.object(m,'boot',side_effect=lambda _e,_h,_p,action,*a,**k:trace.append(action)),patch.object(m,'rollback_services',side_effect=lambda *a,**k:trace.append('switch')):
                m.rollback(home/'entry',home)
            self.assertEqual(trace,['probe','switch'])
    def test_live_unknown_controller_and_unmanaged_install_link_are_preserved(self):
        from launcher import Process
        with tempfile.TemporaryDirectory() as d:
            m,home,entry,prefix,calls,run=self.fixture(d)
            with self.assertRaisesRegex(RuntimeError,'非托管控制器'):
                m.deploy(entry,home,home/'base',prefix,user='orangepi',units=home/'units',run=run,
                         processes=[Process(9999999,os.getuid(),('/tmp/fc01_system_node',))])
            self.assertFalse(any(x[0]=='sudo' for x in calls));calls.clear()
            (entry/'install').symlink_to(home/'outside')
            with self.assertRaises(RuntimeError):
                m.deploy(entry,home,home/'base',prefix,user='orangepi',units=home/'units',run=run,processes=[])
            self.assertFalse(any(x[0]=='sudo' for x in calls))
    def test_compiler_access_time_does_not_force_a_new_live_binary(self):
        m=self.module()
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);compiler=root/'g++';compiler.write_text('compiler')
            with patch.object(m.shutil,'which',return_value=str(compiler)):
                first=m.source_key(root,root/'base');st=compiler.stat()
                os.utime(compiler,ns=(st.st_atime_ns+10_000_000_000,st.st_mtime_ns))
                self.assertEqual(first,m.source_key(root,root/'base'))
                os.utime(compiler,ns=(st.st_atime_ns,st.st_mtime_ns+1))
                self.assertNotEqual(first,m.source_key(root,root/'base'))
    def test_installed_tool_source_changes_select_a_new_build_key(self):
        m=self.module()
        with tempfile.TemporaryDirectory() as d:
            entry=Path(d);tool=entry/'native/tools/vision_odometry_adapter.py'
            tool.parent.mkdir(parents=True);tool.write_text('old tool\n')
            before=m.source_key(entry,entry/'sdk')
            # 控制器源码没变，但 CMake 安装的 Python 工具变化也需要重新安装。
            tool.write_text('new tool\n')
            self.assertNotEqual(before,m.source_key(entry,entry/'sdk'))
    def test_completion_covers_tools_typesupport_and_generated_messages(self):
        m=self.module()
        installed_names=(
            'lib/jikl_fc01_system/velocity_command_diagnostic.py',
            'lib/libjikl_fc01_system__rosidl_typesupport_cpp.so',
            'lib/python3.10/site-packages/jikl_fc01_system/msg/_velocity_command.py',
        )
        for changed_name in installed_names:
            for action in ('modify','remove','add'):
                with self.subTest(file=changed_name,action=action),tempfile.TemporaryDirectory() as d:
                    entry=Path(d);prefix=entry/'.build/key/install';commands=[]
                    def run(argv,**kwargs):
                        commands.append(argv)
                        if argv[1]=='--install':
                            # 构建替身只生成安装结果；不会运行控制器或修改服务。
                            for name in (*installed_names,'lib/jikl_fc01_system/fc01_system_node',
                                         'lib/jikl_fc01_system/fc01_config_check'):
                                path=prefix/name;path.parent.mkdir(parents=True,exist_ok=True)
                                path.write_text('original '+name)
                    with patch.object(m.shutil,'which',return_value='/usr/bin/tool'):
                        m.build_native(entry,prefix,entry/'.build/key/build',run=run)
                        self.assertTrue(m.build_complete(prefix))
                        m.build_native(entry,prefix,entry/'.build/key/build',run=run)
                        self.assertEqual(len(commands),3) # 完全一致的安装树仍可复用。
                        changed=prefix/changed_name
                        if action=='modify':changed.write_text('changed artifact')
                        elif action=='remove':changed.unlink()
                        else:changed.with_name(changed.name+'.extra').write_text('unrecorded artifact')
                        self.assertFalse(m.build_complete(prefix))
                        # 不覆盖可能仍被服务使用的旧前缀，重试必须选择独立的新目录。
                        with self.assertRaisesRegex(RuntimeError,'已有构建目录不完整或被修改'):
                            m.build_native(entry,prefix,entry/'.build/key/build',run=run)
                        self.assertNotEqual(m.staging_path(entry,'key'),prefix.parent)
                        self.assertTrue(prefix.is_dir())
    def test_legacy_binary_only_completion_record_is_not_reused(self):
        import hashlib
        m=self.module()
        with tempfile.TemporaryDirectory() as d:
            entry=Path(d);prefix=entry/'.build/key/install';saved={}
            for name in ('fc01_system_node','fc01_config_check'):
                binary=prefix/'lib/jikl_fc01_system'/name
                binary.parent.mkdir(parents=True,exist_ok=True);binary.write_bytes(b'original')
                saved[name]=hashlib.sha256(binary.read_bytes()).hexdigest()
            # 旧证据缺少消息和工具摘要，不能把它升级解释成完整安装验收。
            (prefix.parent/'build-complete.json').write_text(json.dumps(saved))
            self.assertFalse(m.build_complete(prefix))
            self.assertNotEqual(m.staging_path(entry,'key'),prefix.parent)
    def test_partial_install_prefix_is_retained_and_retry_selects_a_new_stage(self):
        m=self.module()
        with tempfile.TemporaryDirectory() as d:
            entry=Path(d);old=entry/'.build/key/install';old.mkdir(parents=True)
            (old/'partial').write_text('preserve')
            chosen=m.staging_path(entry,'key')
            self.assertNotEqual(chosen,old.parent)
            self.assertTrue(str(chosen).startswith(str(entry/'.build/key')))
            self.assertEqual((old/'partial').read_text(),'preserve')
    def test_pending_link_conflict_is_detected_before_disabling_old_service(self):
        with tempfile.TemporaryDirectory() as d:
            m,home,entry,prefix,calls,run=self.fixture(d)
            (entry/'.install-link-new').write_text('foreign')
            with self.assertRaises(RuntimeError):
                m.deploy(entry,home,home/'base',prefix,user='orangepi',units=home/'units',run=run,processes=[])
            self.assertFalse(any(x[0]=='sudo' for x in calls))
    def test_configuration_only_change_restarts_then_identical_config_does_not(self):
        with tempfile.TemporaryDirectory() as d:
            m,home,entry,prefix,calls,run=self.fixture(d)
            m.deploy(entry,home,home/'base',prefix,user='orangepi',units=home/'units',run=run,processes=[])
            folder=home/'FC01_navigation';folder.mkdir(mode=0o700,exist_ok=True)
            prepared=folder/'install-runtime.yaml';live=folder/'runtime.yaml'
            prepared.write_text('target: 0.75\n');live.write_text('target: 0.5\n');calls.clear()
            m.deploy(entry,home,home/'base',prefix,user='orangepi',units=home/'units',run=run,processes=[])
            self.assertTrue(any(' stop fc01-navigation.service' in ' '.join(x) for x in calls))
            live.write_bytes(prepared.read_bytes());calls.clear()
            m.deploy(entry,home,home/'base',prefix,user='orangepi',units=home/'units',run=run,processes=[])
            self.assertFalse(any(' stop fc01-navigation.service' in ' '.join(x) for x in calls))
    def test_authentication_delay_cannot_reuse_prior_ground_evidence(self):
        with tempfile.TemporaryDirectory() as d:
            m,home,entry,prefix,calls,run=self.fixture(d);armed=[False]
            def runner(argv,**kw):
                if argv[0]=='sudo' and '-n' not in argv:armed[0]=True # aircraft arms while password prompt waits
                return run(argv,**kw)
            def observe(*args,**kwargs):
                calls.append(['PROBE']);self.assertTrue(armed[0]);raise RuntimeError('armed after authentication')
            with patch.object(m,'boot',side_effect=observe),self.assertRaisesRegex(RuntimeError,'armed after authentication'):
                m.deploy(entry,home,home/'base',prefix,user='orangepi',units=home/'units',run=runner,processes=[])
            self.assertFalse(any(x[0]=='sudo' and '/usr/bin/systemctl' in x for x in calls))
    def test_ground_evidence_expiry_reprobes_before_next_service_mutation(self):
        with tempfile.TemporaryDirectory() as d:
            m,home,entry,prefix,calls,run=self.fixture(d);clock=[1000.]
            first=m.render_unit('orangepi',home,entry,home/'base');(home/'units'/m.SERVICE).write_text(first)
            probes=[0]
            def observer(_e,_h,_p,action,*a):
                probes[0]+=1;calls.append(['PROBE'])
                if probes[0]>1:raise RuntimeError('ground permit expired and aircraft armed')
                folder=home/'FC01_navigation';folder.mkdir(mode=0o700,exist_ok=True)
                (folder/'probe.json').write_text(json.dumps(dict(ok=True,armed_seen=False,updated_monotonic=clock[0],
                    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())))
            def runner(argv,**kw):
                result=run(argv,**kw)
                if 'disable' in argv:clock[0]+=3
                return result
            with patch.object(m,'boot',side_effect=observer),patch.object(time,'monotonic',side_effect=lambda:clock[0]),\
                 self.assertRaisesRegex(RuntimeError,'expired'):
                m.deploy(entry,home,home/'base',prefix,user='orangepi',units=home/'units',run=runner,processes=[])
            self.assertFalse(any(' stop fc01-navigation.service' in ' '.join(x) for x in calls))
    def test_rollback_authenticates_then_observes_and_uses_noninteractive_mutations(self):
        with tempfile.TemporaryDirectory() as d:
            m,home,entry,prefix,calls,run=self.fixture(d)
            (home/'units'/m.SERVICE).write_text(m.render_unit('orangepi',home,entry,home/'base'))
            m.rollback_services(entry,home,home/'base',units=home/'units',run=run)
            auth=next(i for i,x in enumerate(calls) if x==['sudo','-v'])
            observation=next(i for i,x in enumerate(calls) if x==['PROBE'])
            self.assertLess(auth,observation)
            changes=[x for x in calls if x[0]=='sudo' and '/usr/bin/systemctl' in x]
            self.assertEqual(len(changes),2)
            self.assertTrue(all('-n' in x for x in changes))

class BundleTests(unittest.TestCase):
    def test_candidate_directory_and_development_experiment_exclusion(self):
        b=importlib.import_module('build_navigation_bundle')
        self.assertFalse(any(name.startswith('native/tests/ros_receive_ab/') for name in b.names(ROOT)))
        with tempfile.TemporaryDirectory() as d:
            folder=Path(d);candidate='FC01_OrangePi_Navigation_20261107_feedback'
            script=b.build(folder/'candidate.sh',ROOT,directory_name=candidate)
            env={**os.environ,'HOME':str(folder)};env.pop('FC01_NAVIGATION_INSTALL_DIR',None)
            r=subprocess.run(['bash',str(script),'--install-only'],env=env,capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stdout+r.stderr)
            self.assertTrue((folder/candidate/'native/src/jikl_fc01_system/src/fc01_system_node.cpp').is_file())
            self.assertFalse((folder/b.DEFAULT_DIRECTORY).exists())
            for invalid in ('../live', 'live/path', "live'", ''):
                with self.assertRaises(ValueError):b.build(folder/'bad.sh',ROOT,directory_name=invalid)

    def test_exact_self_extraction_repeat_and_conflict(self):
        b=importlib.import_module('build_navigation_bundle')
        with tempfile.TemporaryDirectory() as d:
            folder=Path(d);script=b.build(folder/'navigation.sh',ROOT);dest=folder/'installed'
            env={**os.environ,'FC01_NAVIGATION_INSTALL_DIR':str(dest)}
            for _ in range(2):
                r=subprocess.run(['bash',str(script),'--install-only'],env=env,capture_output=True,text=True)
                self.assertEqual(r.returncode,0,r.stdout+r.stderr)
            for name in b.names(ROOT):self.assertEqual((dest/name).read_bytes(),(ROOT/name).read_bytes(),name)
            r=subprocess.run(['bash',str(dest/'navigation.sh'),'--help'],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr)
            (dest/'navigation.py').write_text('existing different')
            r=subprocess.run(['bash',str(script),'--install-only'],env=env,capture_output=True,text=True)
            self.assertNotEqual(r.returncode,0);self.assertEqual((dest/'navigation.py').read_text(),'existing different')

if __name__=='__main__':unittest.main()
