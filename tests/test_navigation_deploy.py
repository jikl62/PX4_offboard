# navigation_deploy 的动作编排测试：offline 只做构建/配置，activate 才允许进入显式在线路径。
# 所有阶段通过 FakeLog 记录命令而不执行；临时现场配置用于证明离线准备不会覆盖正在运行的设置。
"""Offline/online orchestration boundaries without ROS, systemd or hardware."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import navigation_deploy as deploy


# 模拟部署日志的 step 接口，保存阶段与参数供顺序和禁用命令断言。
class FakeLog:
    def __init__(self, folder):
        self.path = folder/'offline.log'
        self.commands = []
        self.stage = ''

    def step(self, stage, command):
        self.commands.append((stage, list(map(str, command))))


class DeploymentTests(unittest.TestCase):
    # 复制候选配置和设置到临时源包，再生成实际内容摘要，模拟可追溯发布输入。
    def release(self, root):
        paths = ['native/config/flight_test.yaml', 'navigation_settings.json']
        repo = Path(__file__).resolve().parents[1]
        for name in paths:
            (root/name).parent.mkdir(parents=True, exist_ok=True)
            (root/name).write_bytes((repo/name).read_bytes())
        files = {name: deploy.sha256(root/name) for name in paths}
        (root/'FC01_RELEASE.json').write_text(json.dumps(dict(
            release_name='candidate', version='20261110.7', payload_sha256=files)))

    # 离线只允许 ARM64_BUILD 与 OFFLINE_CONFIG_CHECK，保持现场 runtime 不变和程序解锁关闭。
    def test_offline_only_builds_and_checks_release_local_config(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)/'release';root.mkdir()
            home = Path(folder)/'home';home.mkdir()
            live = home/'FC01_navigation/runtime.yaml';live.parent.mkdir()
            live.write_text('unchanged live settings')
            self.release(root)
            (root/'ARM64_BUILD_EVIDENCE.json').write_text(json.dumps(dict(
                binaries={'fc01_system_node': {'sha256': 'binary-digest'}})))
            log = FakeLog(root)
            deploy.offline(root, home, log)
            self.assertEqual([stage for stage, _ in log.commands], ['ARM64_BUILD', 'OFFLINE_CONFIG_CHECK'])
            self.assertEqual(log.commands[0][1], ['bash', str(root/'navigation_build_fresh.sh')])
            self.assertIn('check-config', log.commands[1][1])
            self.assertFalse(any(arg in ('probe', 'start', 'stop', 'systemctl', 'run')
                                 for _, command in log.commands for arg in command))
            self.assertEqual(live.read_text(), 'unchanged live settings')
            import yaml
            local = yaml.safe_load((root/'deployment/flight-test-runtime.yaml').read_text())['fc01_system']['ros__parameters']
            self.assertIs(local['arm_enabled'], False)
            self.assertEqual(local['rc_target_height'], 1.5)
            self.assertEqual(local['fc01_entry_version'], '20261110.7')
            ready = json.loads((root/'OFFLINE_DEPLOYMENT.json').read_text())
            self.assertFalse(ready['online_probe_executed'] or ready['services_modified'] or ready['controller_executed'])

    # 构建异常后必须删掉旧成功标记，并且不运行配置准备或写新 READY 证据。
    def test_build_failure_never_reaches_config_or_ready(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder);self.release(root)
            (root/'OFFLINE_DEPLOYMENT.json').write_text('{"old_success": true}')
            log = FakeLog(root)
            with patch.object(log, 'step', side_effect=deploy.DeploymentError('compiler failed')), \
                    patch.object(deploy, 'prepare_offline_config') as config:
                with self.assertRaisesRegex(deploy.DeploymentError, 'compiler failed'):
                    deploy.offline(root, root, log)
            config.assert_not_called()
            self.assertFalse((root/'OFFLINE_DEPLOYMENT.json').exists())

    # 源文件摘要不符时，连构建动作都不能开始。
    def test_mutated_source_is_refused_before_any_action(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder);self.release(root)
            (root/'navigation_settings.json').write_text('tampered')
            log = FakeLog(root)
            with self.assertRaisesRegex(deploy.DeploymentError, '摘要'):
                deploy.offline(root, root, log)
            self.assertEqual(log.commands, [])

    # 在线启用同时需要确认和当前清单/构建证据，旧成功文件不能替代。
    def test_activation_needs_confirmation_and_current_offline_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder);self.release(root);log = FakeLog(root)
            with self.assertRaisesRegex(deploy.DeploymentError, 'confirm-flight-test'):
                deploy.online(root, root, log, action='activate')
            (root/'ARM64_BUILD_EVIDENCE.json').write_text('{}')
            (root/'OFFLINE_DEPLOYMENT.json').write_text(json.dumps(dict(
                schema_version=1,
                release_manifest_sha256=deploy.sha256(root/'FC01_RELEASE.json'),
                build_evidence_sha256='stale')))
            with self.assertRaisesRegex(deploy.DeploymentError, '离线构建证据'):
                deploy.online(root, root, log, action='activate', confirm=True)
            self.assertEqual(log.commands, [])

    # 启用保持原开机策略；回滚使用已存备份，不因当前候选损坏而阻止恢复。
    def test_activation_keeps_boot_policy_and_rollback_uses_saved_backup(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder);self.release(root);log = FakeLog(root)
            (root/'ARM64_BUILD_EVIDENCE.json').write_text('{}')
            config = deploy.prepare_offline_config(root)
            (root/'OFFLINE_DEPLOYMENT.json').write_text(json.dumps(dict(
                schema_version=1,validation_config=str(config),validation_config_sha256=deploy.sha256(config),
                release_manifest_sha256=deploy.sha256(root/'FC01_RELEASE.json'),
                build_evidence_sha256=deploy.sha256(root/'ARM64_BUILD_EVIDENCE.json'))))
            backup = root/'backup'
            (root/'ACTIVATION_STATE.json').write_text(json.dumps(dict(backup=str(backup))))
            deploy.online(root, root, log, action='activate', confirm=True)
            self.assertEqual(log.commands[-1][1][-2:], ['--boot-policy', 'preserve'])
            deploy.online(root, root, log, action='rollback')
            self.assertEqual(log.commands[-1][1][-3:], ['rollback', '--backup', str(backup)])
            config.write_text('changed validation input')
            with self.assertRaisesRegex(deploy.DeploymentError, '离线构建证据'):
                deploy.online(root, root, log, action='activate', confirm=True)
            # Damaged candidate source cannot prevent restoring a valid backup.
            (root/'navigation_settings.json').write_text('damaged candidate')
            deploy.online(root, root, log, action='rollback')
            self.assertEqual(log.commands[-1][0], 'SAFE_ROLLBACK')


if __name__ == '__main__':
    unittest.main()
