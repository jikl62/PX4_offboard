"""离线驱动真实 SystemNode 的人工 RC 1.5 m 全流程，逐字节比较基线与重构发布/状态。"""

from collections import Counter
import hashlib
import importlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
BASELINE = '5198d33f25aaaf2abc7e4b078169ae84d5c4b2c4'


def replace_once(source, old, new):
    """只修改已知测试夹具的一处文本；若来源漂移则失败，避免静默空跑或误改生产逻辑。"""
    if source.count(old) != 1:
        raise AssertionError('powered-node fixture expected one match: ' + old[:80])
    return source.replace(old, new, 1)


def powered_harness(recovery):
    """复用消息接收、时钟和临时 IPC 夹具，改为冷启动及可控垂向测量；不替换控制算法。"""
    source = recovery.HARNESS.split('\nint main(){', 1)[0]
    source = source.split('\n  void remain_faulted(', 1)[0] + '\n};\n'
    old = '''    // Start with a real RC-authorized attempt and observe actual PX4 departure.
    Flight flight(production_no_vision_rc_config());flight.join();
    flight.s.feedback.offboard=false;flight.s.nav_state=15;
    flight.s.attitude.rc_mode_slot=6;++flight.s.attitude.operator_switch_generation;flight.tick();
    assert(flight.r.attitude_core().reason()=="OPERATOR_TAKEOVER_LATCHED");
    config=flight.c;now=flight.t;stamp=flight.s.status_stamp;
    recovery_test_steady=now;recovery_test_wall_ns=std::int64_t(stamp)*1000;
    node.config_=config;node.runtime_=std::make_unique<fc01::Runtime>(std::move(flight.r));'''
    new = '''    // 从生产候选的冷启动Runtime开始；RC授权必须由后续真实接收/step路径重新建立。
    config=production_no_vision_rc_config();now=0.;stamp=1000000;
    recovery_test_steady=now;recovery_test_wall_ns=std::int64_t(stamp)*1000;
    node.config_=config;node.runtime_=std::make_unique<fc01::Runtime>(config);
    node.runtime_->require_attitude_sink_commit();'''
    source = replace_once(source, old, new)
    for old, new in (
        ('struct RecoveryNode {', 'struct PoweredNode {'),
        ('  RecoveryNode(){', '  PoweredNode(){'),
        ('  ~RecoveryNode(){', '  ~PoweredNode(){'),
        (
            '  bool armed{},offboard{},landed{true};',
            '  bool armed{},offboard{},landed{true},rest_input{true};\n'
            '  double measured_z{2.},measured_vz{};\n'
            '  std::uint8_t mode_input{6},arm_input{3};',
        ),
        (
            'value.timestamp=value.timestamp_sample=raw;value.z=2;',
            'value.timestamp=value.timestamp_sample=raw;value.z=float(measured_z);',
        ),
        ('value.vz=.09f;value.z_deriv=-.15f;', 'value.vz=value.z_deriv=float(measured_vz);'),
        (
            'status.pre_flight_checks_pass=status.power_input_valid=true;',
            'status.pre_flight_checks_pass=status.power_input_valid=true;'
            'status.armed_time=armed?10:0;status.latest_arming_reason=armed?2:0;',
        ),
        ('land.landed=land.at_rest=landed;', 'land.landed=landed;land.at_rest=rest_input;'),
        (
            'switches.mode_slot=6;switches.arm_switch=switches.kill_switch=3;',
            'switches.mode_slot=mode_input;switches.arm_switch=arm_input;switches.kill_switch=3;',
        ),
        ('local(stamp-25000);local(stamp);', 'local(stamp);'),
        ('    assert(node.command_pub_->calls==0);', ''),
        ('    assert(!fc01::attitude::nonzero(node.last_attitude_output_.thrust));', ''),
    ):
        source = replace_once(source, old, new)
    # 以下只是确定性外部输入和只读输出；每次推进均调用真实 SystemNode.tick。
    return (
        source
        + r'''
#include <iomanip>
#include <set>
using Phase=fc01::attitude::Phase;
int main(){
  std::cout<<std::hexfloat;
  PoweredNode n;
  unsigned step=0,targets=0,heartbeats=0,commands=0;
  bool nonzero_seen=false,disarm_sent=false;
  std::set<Phase> phases;
  n.node.attitude_pub_->record=[&](const AttitudeTarget& value){
    ++targets;nonzero_seen=nonzero_seen||value.thrust_body[2]!=0;
    std::cout<<"POWERED TARGET "<<step<<':'<<value.timestamp<<':'<<value.yaw_sp_move_rate;
    for(float x:value.q_d)std::cout<<':'<<x;
    for(float x:value.thrust_body)std::cout<<':'<<x;
    std::cout<<':'<<value.reset_integral<<':'<<value.fw_control_yaw_wheel<<'\n';
  };
  n.node.mode_pub_->record=[&](const Mode& value){
    ++heartbeats;
    std::cout<<"POWERED HEARTBEAT "<<step<<':'<<value.timestamp<<':'
      <<value.position<<':'<<value.velocity<<':'<<value.acceleration<<':'<<value.attitude
      <<':'<<value.body_rate<<':'<<value.thrust_and_torque<<':'<<value.direct_actuator<<'\n';
  };
  n.node.command_pub_->record=[&](const Command& value){
    ++commands;
    std::cout<<"POWERED COMMAND "<<step<<':'<<value.timestamp<<':'<<value.command
      <<':'<<value.param1<<':'<<value.param2<<':'<<value.param3<<':'<<value.param4
      <<':'<<value.param5<<':'<<value.param6<<':'<<value.param7
      <<':'<<unsigned(value.target_system)<<':'<<unsigned(value.target_component)
      <<':'<<unsigned(value.source_system)<<':'<<unsigned(value.source_component)
      <<':'<<unsigned(value.confirmation)<<':'<<value.from_external<<'\n';
    // 人工RC主链不能由本地节点自动解锁或自动进入Offboard。
    assert(value.command==400&&value.param1==0);
    disarm_sent=true;
  };
  const auto advance=[&](){
    n.cycle();
    phases.insert(n.node.runtime_->attitude_core().phase());
    std::cout<<"POWERED STATE "<<step<<':'<<n.node.state(n.node.snapshot(n.now),n.now)<<'\n';
    assert(targets==heartbeats);
    ++step;
  };
  for(int i=0;i<90;++i)advance();
  assert(n.node.runtime_->attitude_core().phase()==Phase::RcWaitArm);
  n.arm_input=1;advance();
  n.armed=true;n.rest_input=false;advance();
  n.mode_input=1;advance();
  n.offboard=true;advance();
  assert(n.node.runtime_->attitude_core().phase()==Phase::ArmedGround);
  for(int i=0;i<110;++i)advance();
  assert(n.node.runtime_->attitude_core().phase()==Phase::ThrustRamp);
  assert(!n.node.runtime_->attitude_core().liftoff_confirmed());
  const double reference=n.node.runtime_->attitude_core().reference_z();
  double height=0.;n.landed=false;
  for(int i=0;i<50&&n.node.runtime_->attitude_core().phase()==Phase::ThrustRamp;++i){
    height+=.005;n.measured_z=reference-height;n.measured_vz=-.10;advance();
  }
  assert(n.node.runtime_->attitude_core().phase()==Phase::Ascend);
  for(int i=0;i<190&&n.node.runtime_->attitude_core().phase()!=Phase::HeightHold;++i){
    height=std::min(1.5,height+.015);n.measured_z=reference-height;
    n.measured_vz=height<1.5?-.30:0.;advance();
  }
  assert(n.node.runtime_->attitude_core().phase()==Phase::HeightHold);
  assert(n.node.runtime_->attitude_core().target_height_settled());
  assert(n.node.runtime_->attitude_core().navigation_allowed());
  assert(n.node.last_attitude_output_.target_height==1.5);
  for(int i=0;i<5;++i)advance();
  auto snapshot=n.node.snapshot(n.now);
  assert(n.node.runtime_->request("LAND",{},snapshot,n.now).accepted);
  for(int i=0;i<300&&n.node.runtime_->attitude_core().phase()!=Phase::SafeDisarmed;++i){
    height=std::max(0.,height-.01);n.measured_z=reference-height;
    n.measured_vz=height>0?.20:0.;
    if(height<=0.)n.landed=n.rest_input=true;
    advance();
    if(disarm_sent&&n.armed){
      n.node.runtime_->ack({n.node.runtime_->pending(400).wire_stamp+1,400,0,1,191,false},n.now+.001);
      n.armed=false;
    }
  }
  assert(n.node.runtime_->attitude_core().phase()==Phase::SafeDisarmed);
  // 真实接收路径先观察这次外部开关/模式接管，不能用纯Core夹具的直接Idle假定绕过事件锁存。
  n.offboard=false;n.mode_input=6;n.arm_input=3;advance();
  assert(n.node.runtime_->attitude_core().phase()==Phase::Safety);
  assert(n.node.runtime_->attitude_core().reason()=="OPERATOR_TAKEOVER_LATCHED");
  assert(!n.node.last_attitude_output_.publish);
  for(int i=0;i<200&&n.node.runtime_->attitude_core().phase()!=Phase::RcWaitArm;++i)advance();
  assert(n.node.runtime_->attitude_core().phase()==Phase::RcWaitArm);
  assert(!n.node.runtime_->attitude_core().flight_started());
  assert(!fc01::attitude::nonzero(n.node.last_attitude_output_.thrust));
  assert(phases.count(Phase::ThrustRamp)&&phases.count(Phase::Ascend)&&
    phases.count(Phase::HeightHold)&&phases.count(Phase::Descent));
  assert(nonzero_seen&&disarm_sent&&commands==1&&targets==heartbeats);
  assert(n.config.attitude.rc_target_height==1.5&&n.config.attitude.vertical.hover==.50&&
    n.config.attitude.takeoff_reserve_thrust==.04&&n.config.attitude.vertical.maximum==.80&&
    !n.config.attitude.visual.enabled&&!n.config.attitude.arm_enabled);
  std::cout<<"POWERED COMPLETE "<<step<<':'<<targets<<':'<<heartbeats<<':'<<commands<<'\n';
}
'''
    )


class PoweredNodeEquivalenceTests(unittest.TestCase):
    """以真实节点完成非零 RC 起飞、1.5 m 保持、下降、自动上锁与人工模式释放。"""

    def test_powered_rc_lifecycle_and_all_sink_fields_match_baseline(self):
        """只比较临时 fake-ROS 可执行程序；完整状态字节与每个 sink 字段均严格相同。"""
        git = shutil.which('git')
        if git is None:
            self.skipTest('historical differential requires Git')
        found = subprocess.run(
            [git, 'rev-parse', '--verify', BASELINE + '^{commit}'],
            cwd=ROOT,
            capture_output=True,
            text=True,
        )
        if found.returncode:
            self.fail('required baseline object unavailable: ' + found.stderr)
        self.assertEqual(found.stdout.strip(), BASELINE)
        sys.path.insert(0, str(ROOT / 'native/tests'))
        self.addCleanup(lambda: sys.path.remove(str(ROOT / 'native/tests')))
        recovery = importlib.import_module('test_rc_recovery_adapter')
        with tempfile.TemporaryDirectory(prefix='fc01_stage2_powered_node_') as directory:
            folder = Path(directory)
            baseline = folder / 'baseline'
            baseline.mkdir()
            archive = subprocess.check_output([git, 'archive', BASELINE, 'native'], cwd=ROOT)
            with tarfile.open(fileobj=io.BytesIO(archive)) as source:
                for item in source.getmembers():
                    if not (item.isfile() or item.isdir()) or '..' in Path(item.name).parts:
                        raise AssertionError('unexpected baseline archive member')
                source.extractall(baseline)
            evidence = Path(os.environ.get('FC01_STAGE2_EVIDENCE', str(folder / 'evidence')))
            evidence.mkdir(parents=True, exist_ok=True)
            records = []
            harness = powered_harness(recovery)
            for label, tree in [('baseline', baseline), ('current', ROOT)]:
                # 基线源码已存在该目录，因此编译输入另放；0700 保留真实 UnixServer 目录所有权门。
                build = folder / ('build-' + label)
                build.mkdir(mode=0o700)
                with patch.object(recovery.fixture, 'ROOT', tree / 'native'):
                    recovery.write_fixture(build)
                adapter = build / 'adapter_under_test.cpp'
                # 唯一规范化是诊断 JSON 的 PID 文本；真实 getpid/锁/父进程安全检查仍照常执行。
                adapter.write_text(
                    replace_once(
                        adapter.read_text(),
                        r'out << ",\"controller_pid\":" << getpid()',
                        r'out << ",\"controller_pid\":" << 4242',
                    )
                )
                header = build / 'rclcpp/rclcpp.hpp'
                header.write_text(
                    replace_once(
                        header.read_text(),
                        'unsigned calls{};void publish(const T&){++calls;};',
                        'unsigned calls{};std::function<void(const T&)> record;'
                        'void publish(const T& value){++calls;if(record)record(value);};',
                    )
                )
                command = build / 'px4_msgs/msg/vehicle_command.hpp'
                command.write_text(
                    replace_once(
                        command.read_text(),
                        'float param1{},param2{};',
                        'float param1{},param2{},param3{},param4{},param7{};'
                        'double param5{},param6{};std::uint8_t confirmation{};',
                    )
                )
                driver = build / 'powered.cpp'
                driver.write_text(harness)
                binary = build / 'powered'
                argv = [
                    'g++',
                    '-std=c++17',
                    '-Wall',
                    '-Wextra',
                    '-Werror',
                    '-pedantic',
                    '-pthread',
                    '-I',
                    str(build),
                    '-I',
                    str(tree / 'native/src/jikl_fc01_system/include'),
                    '-I',
                    str(tree / 'native/tests'),
                    str(driver),
                    '-o',
                    str(binary),
                ]
                built = subprocess.run(argv, capture_output=True, text=True, timeout=120)
                self.assertEqual(built.returncode, 0, built.stdout + built.stderr)
                result = subprocess.run(
                    [str(binary)],
                    env=dict(os.environ, FC01_SOCKET=str(build / 'control.sock')),
                    capture_output=True,
                    timeout=30,
                )
                (evidence / ('powered-node-' + label + '-stderr.log')).write_bytes(result.stderr)
                (evidence / ('powered-node-' + label + '-full.log')).write_bytes(result.stdout)
                self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))
                trace = b'\n'.join(
                    line for line in result.stdout.splitlines() if line.startswith(b'POWERED ')
                )
                (evidence / ('powered-node-' + label + '.log')).write_bytes(trace)
                records.append(trace)
            self.assertEqual(records[0], records[1], 'powered node behavior differs from 5198d33')
            states = [
                json.loads(line.split(b':', 1)[1])
                for line in records[1].splitlines()
                if line.startswith(b'POWERED STATE ')
            ]
            phases = Counter(row['phase'] for row in states)
            self.assertTrue({'THRUST_RAMP', 'ASCEND', 'HEIGHT_HOLD', 'DESCENT'} <= set(phases))
            for phase in ('THRUST_RAMP', 'ASCEND', 'HEIGHT_HOLD', 'DESCENT'):
                self.assertTrue(
                    any(
                        row['phase'] == phase and row.get('armed') and row.get('nav_state') == 14
                        for row in states
                    ),
                    phase,
                )
            self.assertTrue(
                any(
                    type(row.get('last_target_local_thrust_z')) in (int, float)
                    and row['last_target_local_thrust_z'] < 0
                    for row in states
                )
            )
            self.assertTrue(records[1].splitlines()[-1].startswith(b'POWERED COMPLETE '))
            sink_order = {}
            for line in records[1].decode().splitlines():
                if line.startswith(('POWERED TARGET ', 'POWERED HEARTBEAT ', 'POWERED COMMAND ')):
                    _, kind, payload = line.split(' ', 2)
                    sink_order.setdefault(int(payload.split(':', 1)[0]), []).append(kind)
            self.assertTrue(sink_order)
            for order in sink_order.values():
                self.assertIn(order, [['TARGET', 'HEARTBEAT'], ['TARGET', 'HEARTBEAT', 'COMMAND']])
            summary = {
                'baseline': BASELINE,
                'comparison': 'strict-byte-identical',
                'driver_sha256': hashlib.sha256(harness.encode()).hexdigest(),
                'state_samples': len(states),
                'phase_counts': dict(phases),
                'sink_order_counts': {
                    ','.join(order): count
                    for order, count in Counter(tuple(row) for row in sink_order.values()).items()
                },
                'trace_sha256': hashlib.sha256(records[1]).hexdigest(),
                'normalization': {'diagnostic_controller_pid': 4242},
                'float_sink_encoding': 'hexfloat exact for all target/command floating fields',
                'scope': 'Synthetic 20Hz source feedback and fake ROS; production node/control/publish code.',
                'hardware_or_flight_validation': False,
            }
            (evidence / 'powered-node.json').write_text(json.dumps(summary, indent=2) + '\n')


if __name__ == '__main__':
    unittest.main()
