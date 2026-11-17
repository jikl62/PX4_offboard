# 权限状态与控制台边界回归：编译真实Runtime快照工具，并验证控制台命令的前置拒绝。
# 全部使用临时主机二进制或mock socket，不授予实际飞行权限。
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import unittest
import json
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


# 从工程内真实fcctl模块加载解析器，避免测到另一份已安装工具。
def console():
    spec = importlib.util.spec_from_file_location('i_fcctl', ROOT / 'tools/fcctl.py')
    obj = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(obj)
    return obj


# 覆盖状态序列化、命令语法与连接之前的权限变更限制。
class AuthorityToolsTests(unittest.TestCase):
    # 读取初始、选择、就绪、Kill观察和故障五个快照，区别观测到的接管原因与内部故障锁存。
    def test_actual_runtime_authority_json(self):
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / 'status'
            subprocess.run(
                [
                    'g++',
                    '-std=c++17',
                    '-Wall',
                    '-Wextra',
                    '-Werror',
                    '-pedantic',
                    '-I',
                    str(ROOT / 'src/jikl_fc01_system/include'),
                    str(ROOT / 'tests/authority_status_dump.cpp'),
                    '-o',
                    str(exe),
                ],
                check=True,
            )
            items = [
                json.loads(line)
                for line in subprocess.check_output([str(exe)], text=True).splitlines()
            ]
        initial, selected, ready, killed, faulted = items
        self.assertEqual(initial['authority_mode'], 'UNSET')
        self.assertEqual(initial['authority_reason'], 'AUTHORITY_MODE_UNSET')
        self.assertEqual(selected['authority_mode'], 'AUTONOMOUS_NO_RC')
        self.assertEqual(selected['authority_reason'], 'AUTONOMOUS_FAILSAFE_UNCONFIRMED')
        for item in [initial, selected]:
            for field in [
                'authority_locked',
                'authority_ready',
                'rc_link_ready',
                'rc_takeover_ready',
                'rc_takeover_confirmed',
                'autonomous_failsafe_ready',
                'autonomous_failsafe_confirmed',
                'operator_takeover_latched',
                'rc_source_seen',
                'rc_source_valid',
                'rc_source_selected',
                'rc_sticks_moving',
                'rc_switches_seen',
                'rc_switches_fresh',
                'rc_switches_ready',
                'rc_kill_latched',
            ]:
                self.assertIs(item[field], False)
            for field in ['rc_data_source', 'rc_mode_slot', 'rc_arm_switch', 'rc_kill_switch']:
                self.assertEqual(item[field], 0)
        self.assertIs(ready['authority_ready'], True)
        self.assertEqual(ready['authority_reason'], 'AUTHORITY_READY')
        self.assertIs(killed['authority_ready'], False)
        self.assertEqual(killed['authority_reason'], 'OPERATOR_TAKEOVER_LATCHED')
        # Kill尚在本轮观测阶段时，原因可先显示接管，但内部operator锁存须到step后才置位。
        self.assertIs(killed['operator_takeover_latched'], False)  # pre-step observed fact
        self.assertIs(faulted['authority_ready'], False)
        self.assertIs(faulted['operator_takeover_latched'], True)
        self.assertEqual(faulted['authority_reason'], 'AUTHORITY_SAFETY_RESTART_REQUIRED')

    # Removing the pre-connect one-shot guard would hit a socket side effect.
    # 确认五种权限子命令精确映射到协议操作码，缺参数或多余参数应报错。
    def test_authority_console_syntax(self):
        ctl = console()
        for text, op in [
            ('authority status', 'AUTHORITY_STATUS'),
            ('authority rc', 'AUTHORITY_RC'),
            ('authority autonomous', 'AUTHORITY_AUTONOMOUS'),
            ('authority reset', 'AUTHORITY_RESET'),
            ('authority takeover', 'AUTHORITY_TAKEOVER'),
        ]:
            self.assertEqual(ctl.parse_command(text), (op, [], 0.0))
        for text in ['authority', 'authority 1', 'authority rc extra', 'authority confirm']:
            with self.assertRaises(ValueError):
                ctl.parse_command(text)

    # 一次性权限变更须在创建socket之前失败；mock一旦被触达就立即报错。
    def test_authority_changes_one_shot_never_connect(self):
        ctl = console()
        for text in [
            'authority rc',
            'authority autonomous',
            'authority reset',
            'authority takeover',
        ]:
            args = SimpleNamespace(command=text, socket='/must/not/connect', json=True, timeout=1.0)
            with patch.object(
                ctl.socket, 'socket', side_effect=AssertionError('socket reached')
            ) as sock:
                with self.assertRaisesRegex(ValueError, 'live interactive console'):
                    ctl.run(args)
                sock.assert_not_called()


if __name__ == '__main__':
    unittest.main()
