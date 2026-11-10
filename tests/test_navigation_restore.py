import importlib
import hashlib
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch


class RestoreTests(unittest.TestCase):
    def module(self):
        return importlib.import_module('navigation_restore')

    def fixture(self, temp):
        m = self.module()
        home = Path(temp)
        entry = home/'new'
        entry.mkdir()
        units = home/'units'
        units.mkdir()
        binary = entry/'install/lib/jikl_fc01_system/fc01_system_node'
        binary.parent.mkdir(parents=True)
        binary.write_bytes(b'controller-test')
        agent = types.SimpleNamespace(pid=31)
        calls = []
        info = {'MainPID':'0', 'ActiveState':'inactive', 'UnitFileState':'disabled',
                'FragmentPath':str(units/m.SERVICE), 'DropInPaths':''}
        def run(argv, **kwargs):
            calls.append(argv)
            if '/usr/bin/install' in argv:
                Path(argv[-1]).write_bytes(Path(argv[-2]).read_bytes())
            if '/usr/bin/rm' in argv:
                Path(argv[-1]).unlink()
            return types.SimpleNamespace(stdout='')
        return m, home, entry, units, binary, agent, calls, info, run

    def test_configure_enables_boot_standby_without_starting_controller_or_agent(self):
        with tempfile.TemporaryDirectory() as temp:
            m, home, entry, units, binary, agent, calls, info, run = self.fixture(temp)
            old = m.render_unit('orangepi', home, home/'old', home/'sdk')
            (units/m.SERVICE).write_text(old)
            def boot(*args):
                calls.append(['BOOT', args[3], *map(str, args[4:])])
            with patch.object(m, 'stopped_services', return_value={m.SERVICE:info}), \
                 patch.object(m, 'unit_info', return_value=info), \
                 patch.object(m, 'existing_agent', return_value=(binary, agent)), \
                 patch.object(m, 'boot', side_effect=boot), \
                 patch('navigation_install.ground_permit_fresh', return_value=True):
                backup = m.configure(entry, home, home/'sdk', units=units, run=run)
            installed = (units/m.SERVICE).read_text()
            self.assertIn(str(entry/'navigation_boot.sh'), installed)
            self.assertIn('run --mode flight-test --confirm-flight-test', installed)
            self.assertIn('Restart=on-failure', installed)
            self.assertIn('StartLimitIntervalSec=120s', installed)
            self.assertIn('StartLimitBurst=5', installed)
            self.assertEqual((backup/m.SERVICE).read_text(), old)
            self.assertEqual(binary.read_bytes(), b'controller-test')
            self.assertIn(['sudo','-n','/usr/bin/systemctl','enable',m.SERVICE], calls)
            self.assertFalse(any(x in ('start','stop','kill','disable','build','--now') for a in calls for x in a))
            auth = calls.index(['sudo','-v'])
            probe = next(i for i, a in enumerate(calls) if a[:2] == ['BOOT','probe'])
            install = next(i for i, a in enumerate(calls) if '/usr/bin/install' in a)
            self.assertLess(auth, probe)
            self.assertLess(probe, install)
            manifest=json.loads((backup/'manifest.json').read_text())
            self.assertEqual(manifest['agent_pid'], 31)
            self.assertEqual(manifest['boot_policy'], 'enable')
            self.assertEqual(manifest['original_unit_file_state'], 'disabled')
            self.assertEqual(manifest['controller_sha256'], hashlib.sha256(binary.read_bytes()).hexdigest())

    def test_missing_ground_evidence_leaves_unit_unchanged(self):
        with tempfile.TemporaryDirectory() as temp:
            m, home, entry, units, binary, agent, calls, info, run = self.fixture(temp)
            with patch.object(m, 'stopped_services', return_value={m.SERVICE:info}), \
                 patch.object(m, 'existing_agent', return_value=(binary, agent)), \
                 patch.object(m, 'boot'), \
                 patch('navigation_install.ground_permit_fresh', return_value=False), \
                 self.assertRaisesRegex(m.ControlError, '接地证据'):
                m.configure(entry, home, home/'sdk', units=units, run=run)
            self.assertFalse((units/m.SERVICE).exists())
            self.assertFalse(any('/usr/bin/install' in a for a in calls))

    def test_running_service_and_unknown_controller_refuse_before_mutation(self):
        m = self.module()
        inactive = {'MainPID':'0','ActiveState':'inactive','DropInPaths':''}
        for running, processes in ((True, []), (False, [types.SimpleNamespace(argv=('/bin/fc01_system_node',))])):
            with patch.object(m, 'unit_info', return_value={**inactive, **({'MainPID':'31','ActiveState':'active'} if running else {})}), \
                 patch.object(m, 'process_list', return_value=processes), \
                 self.assertRaises(m.ControlError):
                m.stopped_services(Path('/unused'), lambda *a, **k: None)

    def test_boot_enabled_old_standby_refuses_without_mutation(self):
        m=self.module()
        def info(name, run):
            return {'MainPID':'0','ActiveState':'inactive','DropInPaths':'',
                    'UnitFileState':'enabled' if name == m.OLD else 'disabled'}
        with patch.object(m,'unit_info',side_effect=info), \
             patch.object(m,'process_list',return_value=[]), \
             self.assertRaisesRegex(m.ControlError,'sudo systemctl disable fc01-standby.service'):
            m.stopped_services(Path('/unused'), lambda *a, **k: None)

    def test_missing_agent_is_not_started_and_external_binary_is_rejected(self):
        m = self.module()
        with tempfile.TemporaryDirectory() as temp:
            entry=Path(temp)/'new';entry.mkdir()
            binary=entry/'node';binary.write_bytes(b'test')
            with patch.object(m, 'check_controller', return_value=binary), \
                 patch.object(m, 'preflight', return_value=(binary,None)), \
                 self.assertRaisesRegex(m.ControlError, '既有 Agent'):
                m.existing_agent(entry, Path(temp), Path(temp)/'sdk')
            with patch.object(m, 'check_controller', return_value=Path('/outside/node')), \
                 self.assertRaisesRegex(m.ControlError, '其他目录'):
                m.existing_agent(entry, Path(temp), Path(temp)/'sdk')

    def test_rollback_restores_unit_without_starting_old_controller(self):
        with tempfile.TemporaryDirectory() as temp:
            m, home, entry, units, binary, agent, calls, info, run = self.fixture(temp)
            original=m.render_unit('orangepi',home,home/'old',home/'sdk')
            current=m.render_unit('orangepi',home,entry,home/'sdk','flight-test',True)
            (units/m.SERVICE).write_text(current)
            backup=home/'FC01_navigation/service-backups/one';backup.mkdir(parents=True)
            (backup/m.SERVICE).write_text(original)
            (backup/'manifest.json').write_text(json.dumps({'service':m.SERVICE,'original_present':True,
                'unit_sha256':hashlib.sha256(current.encode()).hexdigest(),
                'original_sha256':hashlib.sha256(original.encode()).hexdigest()}))
            with patch.object(m, 'stopped_services'), patch.object(m, 'unit_info', return_value=info), \
                 patch.object(m, 'boot'), patch('navigation_install.ground_permit_fresh', return_value=True):
                m.rollback(entry,home,home/'sdk',backup,units=units,run=run)
            self.assertEqual((units/m.SERVICE).read_text(),original)
            self.assertFalse(any(x in ('start','enable','kill') for a in calls for x in a))

    def test_configure_never_disables_previously_enabled_unit(self):
        with tempfile.TemporaryDirectory() as temp:
            m, home, entry, units, binary, agent, calls, info, run = self.fixture(temp)
            info = {**info, 'UnitFileState':'enabled'}
            (units/m.SERVICE).write_text(m.render_unit('orangepi', home, home/'old', home/'sdk'))
            with patch.object(m, 'stopped_services', return_value={m.SERVICE:info}), \
                 patch.object(m, 'unit_info', return_value=info), \
                 patch.object(m, 'existing_agent', return_value=(binary, agent)), \
                 patch.object(m, 'boot'), \
                 patch('navigation_install.ground_permit_fresh', return_value=True):
                m.configure(entry, home, home/'sdk', units=units, run=run)
            self.assertIn(['sudo','-n','/usr/bin/systemctl','enable',m.SERVICE], calls)
            self.assertFalse(any(x in ('disable', '--now', 'start', 'stop') for a in calls for x in a))

    def test_preserve_policy_leaves_persistent_enablement_untouched(self):
        for state in ('enabled', 'disabled', 'enabled-runtime'):
            with self.subTest(state=state), tempfile.TemporaryDirectory() as temp:
                m, home, entry, units, binary, agent, calls, info, run = self.fixture(temp)
                info = {**info, 'UnitFileState':state}
                with patch.object(m, 'stopped_services', return_value={m.SERVICE:info}), \
                     patch.object(m, 'unit_info', return_value=info), \
                     patch.object(m, 'existing_agent', return_value=(binary, agent)), \
                     patch.object(m, 'boot'), \
                     patch('navigation_install.ground_permit_fresh', return_value=True):
                    m.configure(entry, home, home/'sdk', boot_policy='preserve', units=units, run=run)
                self.assertFalse(any(x in ('disable', 'enable', '--now', 'start', 'stop') for a in calls for x in a))

    def test_rollback_restores_original_boot_policy_without_starting(self):
        for state, action in (('enabled','enable'), ('disabled','disable'), ('enabled-runtime','enable')):
            with self.subTest(state=state), tempfile.TemporaryDirectory() as temp:
                m, home, entry, units, binary, agent, calls, info, run = self.fixture(temp)
                original=m.render_unit('orangepi',home,home/'old',home/'sdk')
                current=m.render_unit('orangepi',home,entry,home/'sdk','flight-test',True)
                (units/m.SERVICE).write_text(current)
                backup=home/'FC01_navigation/service-backups/one';backup.mkdir(parents=True)
                (backup/m.SERVICE).write_text(original)
                (backup/'manifest.json').write_text(json.dumps({'service':m.SERVICE,'original_present':True,
                    'unit_sha256':hashlib.sha256(current.encode()).hexdigest(),
                    'original_sha256':hashlib.sha256(original.encode()).hexdigest(),
                    'original_unit_file_state':state}))
                with patch.object(m, 'stopped_services'), patch.object(m, 'unit_info', return_value=info), \
                     patch.object(m, 'boot'), patch('navigation_install.ground_permit_fresh', return_value=True):
                    m.rollback(entry,home,home/'sdk',backup,units=units,run=run)
                self.assertTrue(any(action in a for a in calls))
                self.assertFalse(any(x in ('--now', 'start', 'stop') for a in calls for x in a))
                self.assertEqual((units/m.SERVICE).read_text(), original)

    def test_rollback_without_original_unit_disables_before_removing_definition(self):
        with tempfile.TemporaryDirectory() as temp:
            m, home, entry, units, binary, agent, calls, info, run = self.fixture(temp)
            current=m.render_unit('orangepi',home,entry,home/'sdk','flight-test',True)
            (units/m.SERVICE).write_text(current)
            backup=home/'FC01_navigation/service-backups/one';backup.mkdir(parents=True)
            (backup/'manifest.json').write_text(json.dumps({'service':m.SERVICE,'original_present':False,
                'unit_sha256':hashlib.sha256(current.encode()).hexdigest()}))
            with patch.object(m, 'stopped_services'), patch.object(m, 'unit_info', return_value=info), \
                 patch.object(m, 'boot'), patch('navigation_install.ground_permit_fresh', return_value=True):
                m.rollback(entry,home,home/'sdk',backup,units=units,run=run)
            disable=calls.index(['sudo','-n','/usr/bin/systemctl','disable',m.SERVICE])
            remove=next(i for i,a in enumerate(calls) if '/usr/bin/rm' in a)
            self.assertLess(disable, remove)
            self.assertFalse((units/m.SERVICE).exists())
            self.assertFalse(any(x in ('--now', 'start', 'stop') for a in calls for x in a))


if __name__ == '__main__':
    unittest.main()
