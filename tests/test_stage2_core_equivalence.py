"""第二阶段动态对照：真实C++核心/节点配同一确定性输入，安全字段与发布顺序逐字节比较。"""

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


class CoreBehaviorEquivalenceTests(unittest.TestCase):
    """独立编译两份实现；只有缺少历史对象时跳过对照，不跳过原有运行回归。"""

    @classmethod
    def setUpClass(cls):
        """建立只读基线和输出目录；不切换当前分支，不启动ROS或连接任何飞行设备。"""
        cls.temporary = tempfile.TemporaryDirectory(prefix='fc01_stage2_behavior_')
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.work = Path(cls.temporary.name)
        git = shutil.which('git')
        if git is None:
            raise unittest.SkipTest('历史动态对照需要Git；原有纯运行回归仍独立执行')
        explicit = os.environ.get('FC01_STAGE2_BASELINE')
        if explicit:
            cls.baseline = Path(explicit)
            actual = subprocess.check_output(
                [git, '-C', str(cls.baseline), 'rev-parse', 'HEAD'], text=True
            ).strip()
            if actual != BASELINE:
                raise AssertionError('显式基线不是要求的5198d33')
            if subprocess.check_output(
                [git, '-C', str(cls.baseline), 'status', '--porcelain=v1'], text=True
            ):
                raise AssertionError('显式基线工作区有修改，不能用作原始行为')
        else:
            repository = subprocess.run(
                [git, 'rev-parse', '--show-toplevel'], cwd=ROOT, capture_output=True, text=True
            )
            if repository.returncode:
                if 'not a git repository' in repository.stderr:
                    raise unittest.SkipTest('导出源码无Git历史；不能运行历史动态对照')
                raise RuntimeError(repository.stderr)
            probe = subprocess.run(
                [git, 'rev-parse', '--verify', BASELINE + '^{commit}'],
                cwd=ROOT,
                capture_output=True,
                text=True,
            )
            if probe.returncode:
                if 'Needed a single revision' in probe.stderr:
                    raise unittest.SkipTest('当前克隆缺少历史基线对象5198d33')
                raise RuntimeError(probe.stderr)
            archive = subprocess.check_output([git, 'archive', BASELINE, 'native'], cwd=ROOT)
            cls.baseline = cls.work / 'baseline'
            cls.baseline.mkdir()
            with tarfile.open(fileobj=io.BytesIO(archive)) as source:
                for entry in source.getmembers():
                    if not (entry.isfile() or entry.isdir()) or '..' in Path(entry.name).parts:
                        raise AssertionError('基线归档包含非普通源码项')
                source.extractall(cls.baseline)
        cls.evidence = Path(os.environ.get('FC01_STAGE2_EVIDENCE', str(cls.work / 'evidence')))
        cls.evidence.mkdir(parents=True, exist_ok=True)

    def compare_cpp(self, stem):
        """相同trace源码分别用基线/当前头编译，比较完整stdout；只忽略不参与控制的临时路径。"""
        records = []
        source = ROOT / 'native/tests' / (stem + '.cpp')
        staged = self.work / (stem + '.cpp')
        staged.write_bytes(source.read_bytes())
        for label, tree in [('baseline', self.baseline), ('current', ROOT)]:
            binary = self.work / (stem + '-' + label)
            command = [
                'g++',
                '-std=c++17',
                '-Wall',
                '-Wextra',
                '-Werror',
                '-pedantic',
                '-pthread',
                '-I',
                str(tree / 'native/src/jikl_fc01_system/include'),
                '-I',
                str(tree / 'native/tests'),
                str(staged),
                '-o',
                str(binary),
            ]
            built = subprocess.run(command, capture_output=True, text=True, timeout=120)
            self.assertEqual(built.returncode, 0, built.stdout + built.stderr)
            result = subprocess.run([str(binary)], capture_output=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))
            (self.evidence / (stem + '-' + label + '.log')).write_bytes(result.stdout)
            records.append(result.stdout)
        self.assertEqual(records[0], records[1], stem + ': 检出未经解释的行为差异')
        summary = dict(
            baseline=BASELINE,
            source_sha256=hashlib.sha256(staged.read_bytes()).hexdigest(),
            comparison='strict-byte-identical',
            lines=len(records[0].splitlines()),
            bytes=len(records[0]),
            sha256=hashlib.sha256(records[0]).hexdigest(),
        )
        (self.evidence / (stem + '.json')).write_text(json.dumps(summary, indent=2) + '\n')

    def test_control_trace(self):
        """高度/姿态/RC/视觉与多轮恢复使用真实Core，double输出不设容差。"""
        self.compare_cpp('test_stage2_control_trace')

    def test_runtime_ack_and_ipc_trace(self):
        """请求优先级、ACK归属/水位/终态和真实临时Unix会话均逐字节对照。"""
        self.compare_cpp('test_stage2_runtime_trace')

    def test_publication_policy_and_sink_order(self):
        """逐阶段坏证据与微小非零推力，及三次上下文/双sink部分提交严格对照。"""
        self.compare_cpp('test_stage2_publication_trace')

    def test_actual_node_tick_and_status_bytes(self):
        """运行真实SystemNode.tick/snapshot/state配假ROS；只读IPC和有序sink记录严格相同。"""
        sys.path.insert(0, str(ROOT / 'native/tests'))
        self.addCleanup(lambda: sys.path.remove(str(ROOT / 'native/tests')))
        recovery = importlib.import_module('test_rc_recovery_adapter')
        outputs = []
        for label, tree in [('baseline', self.baseline), ('current', ROOT)]:
            folder = self.work / ('node-' + label)
            # 与生产socket所有权合同一致，不受开发机umask=0002影响而生成组可写目录。
            folder.mkdir(mode=0o700)
            with patch.object(recovery.fixture, 'ROOT', tree / 'native'):
                recovery.write_fixture(folder)
            # 两个可执行进程的真实PID必然不同；仅固定临时副本里这个诊断字段的输入。
            # 不替换getpid系统调用，不改变socket锁所有权、父进程保护或任何安全状态字段。
            adapter = folder / 'adapter_under_test.cpp'
            adapter_source = adapter.read_text()
            pid_field = r'out << ",\"controller_pid\":" << getpid()'
            self.assertEqual(adapter_source.count(pid_field), 1)
            adapter.write_text(
                adapter_source.replace(pid_field, r'out << ",\"controller_pid\":" << 4242', 1)
            )
            header = folder / 'rclcpp/rclcpp.hpp'
            source = header.read_text()
            original = 'unsigned calls{};void publish(const T&){++calls;};'
            self.assertEqual(source.count(original), 1)
            source = source.replace(
                original,
                'unsigned calls{};std::function<void(const T&)> record;'
                'void publish(const T& value){++calls;if(record)record(value);};',
            )
            header.write_text(source)
            harness = recovery.HARNESS.split('\nint main(){', 1)[0]
            harness += r'''
// 回放真实节点地面恢复及坏事件；所有发布器都是内存替身，故障仅注入这份临时夹具。
int main(){
  std::cout<<std::hexfloat;
  for(int scenario=0;scenario<9;++scenario){
    RecoveryNode n;bool injected=false;
    n.node.attitude_pub_->record=[&](const AttitudeTarget& target){
      std::cout<<"TRACE TARGET "<<scenario<<':'<<target.timestamp<<':'<<target.yaw_sp_move_rate;
      for(const auto value:target.q_d)std::cout<<':'<<value;
      for(const auto value:target.thrust_body)std::cout<<':'<<value;
      std::cout<<':'<<target.reset_integral<<':'<<target.fw_control_yaw_wheel<<'\n';
      if(!injected&&scenario==6){injected=true;recovery_test_steady+=.5;}
      if(!injected&&scenario==7){injected=true;throw std::runtime_error("fixture-target");}
    };
    n.node.mode_pub_->record=[&](const Mode& mode){
      std::cout<<"TRACE HEARTBEAT "<<scenario<<':'<<mode.timestamp<<':'<<mode.position
        <<mode.velocity<<mode.acceleration<<mode.attitude<<mode.body_rate
        <<mode.thrust_and_torque<<mode.direct_actuator<<'\n';
      if(!injected&&scenario==8){injected=true;throw std::runtime_error("fixture-heartbeat");}
    };
    for(int round=0;round<150;++round){
      n.armed=scenario==3&&round<70;n.offboard=scenario==4&&round<70;
      if(scenario==5&&round>=90)n.measured_q={0,0,0,0};
      const int unsafe=round<35?(scenario==1?1:scenario==2?2:0):0;
      n.cycle(unsafe);
      std::cout<<"TRACE STATE "<<scenario<<':'<<round<<':'
        <<n.node.state(n.node.snapshot(n.now),n.now)<<'\n';
    }
  }
  return 0;
}
'''
            (folder / 'test.cpp').write_text(harness)
            binary = folder / 'trace'
            command = [
                'g++',
                '-std=c++17',
                '-Wall',
                '-Wextra',
                '-Werror',
                '-pedantic',
                '-pthread',
                '-I',
                str(folder),
                '-I',
                str(tree / 'native/src/jikl_fc01_system/include'),
                '-I',
                str(tree / 'native/tests'),
                str(folder / 'test.cpp'),
                '-o',
                str(binary),
            ]
            built = subprocess.run(command, capture_output=True, text=True, timeout=120)
            self.assertEqual(built.returncode, 0, built.stdout + built.stderr)
            env = dict(os.environ, FC01_SOCKET=str(folder / 'control.sock'))
            result = subprocess.run([str(binary)], env=env, capture_output=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))
            trace = b'\n'.join(
                line for line in result.stdout.splitlines() if line.startswith(b'TRACE ')
            )
            self.assertEqual(
                sum(line.startswith(b'TRACE STATE ') for line in trace.splitlines()), 1350
            )
            (self.evidence / ('node-' + label + '.log')).write_bytes(trace)
            outputs.append(trace)
        self.assertEqual(outputs[0], outputs[1], '节点状态字节/发布顺序与基线不一致')
        summary = dict(
            baseline=BASELINE,
            comparison='strict-byte-identical',
            controlled_fixture_input={'diagnostic_controller_pid': 4242},
            state_samples=1350,
            lines=len(outputs[0].splitlines()),
            sha256=hashlib.sha256(outputs[0]).hexdigest(),
        )
        (self.evidence / 'node.json').write_text(json.dumps(summary, indent=2) + '\n')


if __name__ == '__main__':
    unittest.main()
