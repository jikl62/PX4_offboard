# 生成的 Python 会话参数须交由真实 C++ Config::validate 校验，避免两套 Python 规则相互自证。
# 从真实 ROS 参数映射头提取字段，临时编译无 ROS 的 probe；只做配置解析，不启动控制节点。
"""Exercise generated session parameters against the unchanged native validator."""
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

import yaml
from fc01_control import runtime_profile

ROOT = Path(__file__).resolve().parents[1]


# 检查 Python 与原生参数合同一致：诊断/飞行会话可解析，旧 flight 总开关仍被拒绝。
class NativeProfileContract(unittest.TestCase):
    # 优先用当前工作树头文件；编译器/头缺失才显式跳过，映射数量变化则要求人工复核。
    @classmethod
    def setUpClass(cls):
        reference = (
            ROOT.parent
            / 'control_sources/FC01_R632_P0_Attitude_Audit_Candidate_20261019/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE'
        )
        device = Path(
            os.environ.get(
                'FC01_PROJECT_ROOT',
                str(Path.home() / 'FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE'),
            )
        )
        candidates = [ROOT / 'native/src/jikl_fc01_system/include'] + [
            root / 'src/jikl_fc01_system/include' for root in (reference, device)
        ]
        include = next(
            (
                p
                for p in candidates
                if (p / 'jikl_fc01_system/attitude_ros_parameters.hpp').is_file()
            ),
            None,
        )
        if include is None or shutil.which('g++') is None:
            raise unittest.SkipTest(
                'native FC01 headers and g++ required for the native contract probe'
            )
        cls.tmp = tempfile.TemporaryDirectory(prefix='fc01_native_profile_')
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.binary = Path(cls.tmp.name) / 'probe'
        # Derive parameter -> member mappings from the actual ROS adapter; call
        # the actual Config::validate(), not a Python reproduction of its rules.
        header = (include / 'jikl_fc01_system/attitude_ros_parameters.hpp').read_text()
        # 与回放入口一样，仅兼容 token 间排版；不放宽参数/类型/成员或最少映射数量。
        direct = re.findall(
            r'c\.([\w.]+)\s*=\s*node\.declare_parameter\s*<\s*'
            r'(bool|double|std::string|std::int64_t)\s*>\s*\(\s*"([^"]+)"',
            header,
        )
        doubles = re.findall(r'\{\s*"([^"]+)"\s*,\s*&\s*c\.([\w.]+)\s*\}', header)
        if len(direct) < 9 or len(doubles) < 45:
            raise AssertionError('native adapter mapping changed; contract probe must be reviewed')
        lines = ['cfg.profile=p.at("profile");', 'cfg.flight_enabled=boolean("flight_enabled");']
        for member, kind, name in direct:
            value = (
                f'boolean("{name}")'
                if kind == 'bool'
                else (
                    f'std::stod(p.at("{name}"))'
                    if kind == 'double'
                    else (
                        f'std::stoll(p.at("{name}"))'
                        if kind == 'std::int64_t'
                        else f'p.at("{name}")'
                    )
                )
            )
            lines.append(f'if(p.count("{name}"))cfg.attitude.{member}={value};')
        for name, member in doubles:
            lines.append(f'if(p.count("{name}"))cfg.attitude.{member}=std::stod(p.at("{name}"));')
        source = Path(cls.tmp.name) / 'probe.cpp'
        source.write_text(
            '''#include "jikl_fc01_system/runtime.hpp"
#include <iomanip>
#include <iostream>
#include <map>
int main(){
  try{
    std::map<std::string,std::string> p;std::string key,value;
    while(std::cin>>std::quoted(key)>>std::quoted(value))p[key]=value;
    const auto boolean=[&](const char* key){const auto& v=p.at(key);
      if(v!="true"&&v!="false")throw std::invalid_argument("invalid bool");return v=="true";};
    fc01::Config cfg;
'''
            + '\n'.join(lines)
            + '''
    cfg.validate();std::cout<<"NATIVE_CONFIG_VALID\n";return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 2;}
}
'''.replace(
                '"\n"', '"\\n"'
            ).replace(
                'VALID\n"', 'VALID\\n"'
            )
        )
        built = subprocess.run(
            ['g++', '-std=c++17', '-I', str(include), str(source), '-o', str(cls.binary)],
            capture_output=True,
            text=True,
        )
        if built.returncode:
            raise AssertionError('native config probe failed to compile:\n' + built.stderr)

    # 将参数序列化给临时 probe 的 stdin，检查的是实际原生验证返回码。
    def run_native(self, params):
        values = []
        for name, value in params.items():
            text = ('true' if value else 'false') if type(value) is bool else str(value)
            values.append(json.dumps(name) + ' ' + json.dumps(text))
        return subprocess.run(
            [str(self.binary)], input='\n'.join(values), capture_output=True, text=True, timeout=3
        )

    # 读取只供本测试的 baseline YAML 并生成会话副本，原配置不被覆写。
    def parameters(self, flight):
        base = yaml.safe_load((ROOT / 'tests/fixtures/base.yaml').read_text())
        return runtime_profile(base, flight)['fc01_system']['ros__parameters']

    # 显式 flight 会话的参数应满足原生候选合同，但此测试并不表示取得实机飞行许可。
    def test_flight_session_is_accepted_by_actual_native_config(self):
        result = self.run_native(self.parameters(True))
        self.assertEqual(result.returncode, 0, result.stderr)

    # 所有动作门关闭的检查配置也必须是原生可接受配置。
    def test_check_session_is_accepted_by_actual_native_config(self):
        result = self.run_native(self.parameters(False))
        self.assertEqual(result.returncode, 0, result.stderr)

    # 姿态高度模式禁止复用旧 flight_enabled 通用入口，避免绕过专用权限门。
    def test_native_validator_rejects_legacy_flight_flag_in_attitude_mode(self):
        params = self.parameters(False)
        params['flight_enabled'] = True
        result = self.run_native(params)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('attitude_altitude uses separate flight_actions_enabled gates', result.stderr)
