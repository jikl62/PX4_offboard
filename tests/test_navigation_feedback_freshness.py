# 显示层时效合同回归：封套、样本、接收进度和监督快照四种年龄都必须消耗各自 TTL。
# 时间输入为可控数值，真实 IPC 用例只连接临时 peer，不连接 PX4 或运行 Agent。
"""Display expiry follows aircraft samples, including a live but silent IPC."""
import copy
import json
import os
import socket
import subprocess
import sys
import threading
import time
import unittest

import navigation
import navigation_service
import standby


# 构造当前 RC_READY_WAIT_ARM 反馈及各 Topic 的显式源/接收时间，不以缺省零值代替有效性。
def ready_state():
    return dict(kind='state', phase='RC_READY_WAIT_ARM', reason='RC_READY_WAIT_CH5_MANUAL_ARM',
                status_fresh=True, land_fresh=True, local_fresh=True, attitude_fresh=True,
                local_sample_fresh=True, attitude_sample_fresh=True, battery_fresh=True,
                source_clock_valid=True, armed=False, landed=True, at_rest=True, nav_state=15,
                flight_actions_enabled=True, takeoff_enabled=True, rc_navigation_enabled=True,
                arm_enabled=False, flight_enabled=False, source_timing={
                    topic:dict(timeout_s=timeout, source_reason='VALID', receipt_reason='VALID',
                               envelope_age_s=.02, sample_age_s=.02, receipt_age_s=.02,
                               sample_progress_age_s=.02)
                    for topic,timeout in (('status',2.5),('land',2.5),('local',.5),
                                          ('attitude',.5),('flags',2.5),('battery',1.))})


# 逐项破坏年龄、原生拒绝位和传输身份，证明任一坏证据都不能显示解锁许可。
class FeedbackFreshnessTests(unittest.TestCase):
    # 把当前状态包装成 supervisor 行，并用固定 now 消除测试机墙钟速度影响。
    def view(self, state, elapsed=0., **changes):
        row=dict(status='READY_WAIT_ARM', reason='ready', supervisor_pid=10,
                 updated_unix=1000., state=state, state_age_seconds=0., feedback_current=True)
        row.update(changes)
        return navigation.render_status(row, dict(ActiveState='active',MainPID='10'), now=1000.+elapsed)

    # 正例须显示当前反馈与人工 RC 提示，且不能带过期标签。
    def assert_current(self, view):
        self.assertIn('[READY_WAIT_ARM]',view)
        self.assertIn('已准备：CH5 手动解锁',view)
        self.assertIn('实际模式：',view)

    # 反例只允许历史说明，不能保留“已准备”或实际当前模式。
    def assert_history(self, view):
        self.assertNotIn('[READY_WAIT_ARM]',view)
        self.assertNotIn('已准备：CH5 手动解锁',view)
        self.assertNotIn('实际模式：',view)
        self.assertIn('历史',view)

    # 每个 Topic 的封套/样本/接收/进度年龄任一达到边界就失效，不能用更长话题 TTL 代替。
    def test_each_topic_envelope_receipt_and_raw_sample_ages_expire_at_its_ttl(self):
        for topic in ready_state()['source_timing']:
            names=['envelope_age_s','receipt_age_s']
            if topic in ('local','attitude'):names.extend(('sample_age_s','sample_progress_age_s'))
            for name in names:
                with self.subTest(topic=topic,measurement=name):
                    state=ready_state();item=state['source_timing'][topic]
                    item.update(timeout_s=.5);item[name]=.25
                    self.assert_current(self.view(state,.24))
                    self.assert_history(self.view(state,.25))
                    self.assert_history(self.view(state,.3))

    # 原生状态年龄和保存后墙钟流逝叠加，不能被新文件 mtime 重置。
    def test_supervisor_state_age_and_saved_snapshot_elapsed_both_consume_ttl(self):
        state=ready_state()
        self.assert_current(self.view(state,.2,state_age_seconds=.1))
        self.assert_history(self.view(state,.4,state_age_seconds=.1))
        self.assert_history(self.view(state,0.,state_age_seconds=.5))

    # 缺时间、非法数字或原生已判无效时，显示层不得自行恢复有效。
    def test_missing_invalid_or_native_rejected_timing_cannot_prompt_manual_arming(self):
        for value in (None,{},[], 'unknown'):
            with self.subTest(timings=value):
                state=ready_state();state['source_timing']=value
                self.assert_history(self.view(state))
        for topic in ready_state()['source_timing']:
            names=['timeout_s','source_reason','receipt_reason','envelope_age_s','receipt_age_s']
            if topic in ('local','attitude'):names.extend(('sample_age_s','sample_progress_age_s'))
            for name in names:
                invalid=[None,True,'unknown',float('nan'),float('inf')]
                if name in ('receipt_age_s','sample_progress_age_s'):invalid.append(-.01)
                for value in invalid:
                    with self.subTest(topic=topic,field=name,value=value):
                        state=ready_state();state['source_timing'][topic][name]=value
                        self.assert_history(self.view(state))
                state=ready_state();del state['source_timing'][topic][name]
                self.assert_history(self.view(state))
            state=ready_state();del state['source_timing'][topic]
            self.assert_history(self.view(state))
        for changes in (dict(timeout_s=0),dict(timeout_s=-.5),dict(receipt_age_s=-.01),
                        dict(source_reason='SAMPLE_EXPIRED'),dict(receipt_reason='RECEIPT_EXPIRED')):
            with self.subTest(changes=changes):
                state=ready_state();state['source_timing']['local'].update(changes)
                self.assert_history(self.view(state))

    # 即便数字年龄看似新鲜，false fresh 位或断流历史依然禁止解锁提示。
    def test_native_false_flags_and_transport_history_cannot_be_revived_by_valid_timing(self):
        for name in ('status_fresh','land_fresh','local_fresh','attitude_fresh',
                     'local_sample_fresh','attitude_sample_fresh','battery_fresh','source_clock_valid'):
            with self.subTest(flag=name):
                state=ready_state();state[name]=False
                self.assert_history(self.view(state))
        for changes in (dict(feedback_current=False),dict(ipc_error=dict(kind='clean_eof')),
                        dict(retry_in_seconds=5),dict(controller_returncode=0)):
            with self.subTest(changes=changes):
                self.assert_history(self.view(ready_state(),**changes))

    # 原生容许的小未来源戳不能给显示 TTL 额外加寿命。
    def test_native_valid_future_source_timestamp_does_not_extend_display_lifetime(self):
        state=ready_state()
        for topic in ('local','attitude'):
            state['source_timing'][topic].update(envelope_age_s=-.05,sample_age_s=-.05,receipt_age_s=0.)
        self.assert_current(self.view(state,.4))
        self.assert_history(self.view(state,.5))

    # 未来 sample 的时钟偏差不能抵消进度冻结年龄。
    def test_future_sample_clock_skew_cannot_extend_raw_progress_lifetime(self):
        for topic in ('local','attitude'):
            with self.subTest(topic=topic):
                state=ready_state()
                # A raw stamp accepted 100 ms ahead can be only 390 ms old
                # after its last progress was received 490 ms ago.
                state['source_timing'][topic].update(sample_age_s=.39,sample_progress_age_s=.49)
                self.assertTrue(navigation.aircraft_feedback_current(state,.009))
                self.assertFalse(navigation.aircraft_feedback_current(state,.01))
                self.assert_current(self.view(state,.009))
                self.assert_history(self.view(state,.06))

    # 样本过期只撤销飞机状态可信度，当前控制器锁存与恢复原因仍需显示。
    def test_sample_expiry_preserves_current_fault_and_recovery_diagnostics(self):
        state=ready_state();state.update(phase='SAFETY',reason='OPERATOR_TAKEOVER_LATCHED',
                                        rc_first_fault_reason='OPERATOR_TAKEOVER_LATCHED',
                                        rc_recovery_block_reason='WAIT_NEW_LOCAL_SAMPLE')
        view=self.view(state,.6,status='FAULT',state_age_seconds=.1)
        self.assertIn('[FAULT]',view)
        self.assertIn('故障锁存（当前控制器）',view)
        self.assertIn('恢复等待原因：WAIT_NEW_LOCAL_SAMPLE',view)
        self.assertIn('恢复观察（飞控反馈过期）',view)
        self.assertNotIn('已准备：CH5 手动解锁',view)


PEER = '''import json,signal,socket,sys,time,select
sock=socket.socket(fileno=int(sys.argv[1]));scenario=sys.argv[2]
state=json.loads(sys.argv[3]);stopping=False
def stop(*_):
 global stopping
 stopping=True
signal.signal(signal.SIGTERM,stop)
start=time.monotonic();last=-1.;closed=False
while not stopping:
 now=time.monotonic()-start
 if scenario=='disconnect' and now>=.3 and not closed:
  sock.close();closed=True
 if not closed and (last<0 or now-last>=.05):
  paused=(scenario=='short_pause' and .1<now<.3) or (scenario=='long_pause' and .1<now<1.3) or (scenario=='silent' and now>.1)
  if not paused:
   state['local_fresh']=not(scenario=='source_stale' and now>=.3)
   sock.sendall((json.dumps(state)+'\\n').encode());last=now
 if not closed and select.select([sock],[],[],.01)[0]:
  if not sock.recv(4096):break
 else:time.sleep(.01)
if not closed:sock.close()
'''


# 用真实临时 socket 依次制造正常、暂停、静默、过期和断线，验证渲染实时降级。
class LiveMonitorFreshnessTests(unittest.TestCase):
    # 观测只依赖实际反馈到达/源年龄，进程存活和 supervisor 更新不能为旧数据续命。
    def test_normal_paused_silent_stale_and_disconnected_real_ipc_controls(self):
        for scenario,duration in (('normal',1.5),('short_pause',1.5),('long_pause',1.6),
                                  ('silent',3.4),('source_stale',1.2),('disconnect',1.2)):
            with self.subTest(scenario=scenario):
                a,b=socket.socketpair()
                node=subprocess.Popen([sys.executable,'-c',PEER,str(b.fileno()),scenario,json.dumps(ready_state())],
                                      pass_fds=(b.fileno(),),start_new_session=True,
                                      stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
                b.close();stopped=threading.Event();timer=threading.Timer(duration,stopped.set)
                timer.start();rows=[]
                def observe(raw):
                    row=navigation_service.describe_row(raw)
                    row.update(updated_unix=time.time(),supervisor_pid=os.getpid())
                    view=navigation.render_status(row,dict(ActiveState='active',MainPID=str(os.getpid())),
                                                  now=row['updated_unix'])
                    rows.append((copy.deepcopy(row),view))
                try:
                    standby.monitor(standby.Channel(a),node,stopped,observe)
                    _,error=node.communicate(timeout=3)
                    self.assertEqual(node.returncode,0,error.decode())
                finally:
                    timer.cancel();a.close()
                    if node.poll() is None:node.kill();node.wait(timeout=3)
                    if node.stderr is not None:node.stderr.close()
                self.assertTrue(any('已准备：CH5 手动解锁' in view for _,view in rows))
                if scenario in ('normal','short_pause'):
                    live=[view for row,view in rows if row['status']=='READY_WAIT_ARM']
                    self.assertGreaterEqual(len(live),2)
                    self.assertTrue(all('[READY_WAIT_ARM]' in view for view in live))
                elif scenario in ('long_pause','silent'):
                    old=[view for row,view in rows if row.get('feedback_current') is True
                         and row.get('state_age_seconds',0)>.5]
                    self.assertTrue(old)
                    for view in old:
                        self.assertIn('[STALE]',view)
                        self.assertNotIn('已准备：CH5 手动解锁',view)
                        self.assertNotIn('实际模式：',view)
                else:
                    unavailable=[view for row,view in rows if row.get('feedback_current') is False]
                    self.assertTrue(unavailable)
                    self.assertTrue(all('已准备：CH5 手动解锁' not in view for view in unavailable))


if __name__=='__main__':unittest.main()
