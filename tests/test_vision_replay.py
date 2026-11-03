"""No-hardware end-to-end checks: real Runtime, RC admission and both sinks."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / 'vision_replay.py'
FIXTURES = ROOT / 'tests/fixtures/vision'


class VisionReplay(unittest.TestCase):
    def run_replay(self, *args, success=True):
        self.assertTrue(ENTRY.is_file(), 'the no-hardware replay entry point is missing')
        p = subprocess.run([sys.executable, str(ENTRY), *map(str, args)], cwd=ROOT,
                           capture_output=True, text=True, timeout=75)
        if success:
            self.assertEqual(p.returncode, 0, p.stderr)
            return [json.loads(line) for line in p.stdout.splitlines() if line.strip()]
        self.assertNotEqual(p.returncode, 0)
        return p

    def test_ideal_closed_loop_changes_real_attitude_and_reduces_point_error(self):
        rows = self.run_replay('--demo', 'cruise')
        self.assertGreater(len(rows), 50)
        self.assertTrue(all(r['simulation'] and r['runtime_path'] == 'Runtime/RC/Core/dual_sink' for r in rows))
        moving = [r for r in rows if r['active']]
        self.assertTrue(moving)
        self.assertTrue(all(r['phase'] == 'HEIGHT_HOLD' and r['allowed'] for r in moving))
        self.assertTrue(any(abs(r['q'][1]) + abs(r['q'][2]) > .001 for r in moving))
        self.assertLess(moving[-1]['position_error'], moving[0]['position_error'] * .4)
        self.assertTrue(all(r['target_committed'] and r['heartbeat_committed'] for r in moving))
        self.assertTrue(all(r.get('submitted_q') is not None and r.get('submitted_thrust') is not None for r in moving),
                        'replay must expose the actual wire packet accepted by the target sink')
        self.assertEqual(rows[-1]['arm_commands'], 0)
        self.assertEqual(rows[-1]['mode_commands'], 0)

    def test_target_height_changes_original_vertical_controller_thrust(self):
        rows = self.run_replay('--demo', 'height')
        moving = [r for r in rows if r['active']]
        self.assertGreater(max(r['target_height'] for r in moving), 1.55)
        self.assertGreater(max(-r['thrust'][2] for r in moving), min(-r['thrust'][2] for r in moving) + .005)
        self.assertAlmostEqual(rows[0]['baseline_hover_thrust'], .475)
        self.assertAlmostEqual(rows[0]['baseline_height_gain'], .5)

    def test_missing_state_waits_without_fabricating_valid_zero(self):
        rows = self.run_replay(FIXTURES / 'missing.jsonl')
        self.assertTrue(all(not r['active'] and r['phase'] == 'HEIGHT_HOLD' for r in rows))
        self.assertTrue(all(r['state_sequence'] == 0 for r in rows))

    def test_expired_state_enters_native_descent_and_cannot_reactivate(self):
        rows = self.run_replay(FIXTURES / 'state_timeout.jsonl')
        self.assertTrue(rows[0]['active'])
        self.assertTrue(any(r['phase'] in ('DESCENT', 'RECOVERY_DESCENT') for r in rows[1:]))
        first_descent = next(i for i, r in enumerate(rows) if r['phase'] in ('DESCENT', 'RECOVERY_DESCENT'))
        self.assertTrue(all(not r['active'] for r in rows[first_descent:]))
        self.assertTrue(all(r['allowed'] for r in rows))

    def test_bad_frame_and_missing_valid_velocity_latch_native_descent(self):
        for name in ('bad_frame.jsonl', 'missing_velocity.jsonl'):
            with self.subTest(name=name):
                rows = self.run_replay(FIXTURES / name)
                self.assertTrue(rows[0]['active'])
                self.assertIn(rows[-1]['phase'], ('DESCENT', 'RECOVERY_DESCENT'))
                self.assertFalse(rows[-1]['active'])

    def test_follow_axes_and_modes_are_real_motion_requests(self):
        rows = self.run_replay(FIXTURES / 'follow_land.jsonl')
        self.assertGreater(rows[0]['velocity_target'][0], 0)
        self.assertGreater(rows[0]['velocity_target'][1], 0)
        self.assertGreater(rows[0]['yaw'], 0)
        self.assertGreater(rows[0]['target_height'], .5)
        self.assertIn(rows[-1]['phase'], ('DESCENT', 'RECOVERY_DESCENT'))
        self.assertFalse(rows[-1]['active'])

    def test_velocity_feedback_changes_attitude_at_same_position(self):
        rows = self.run_replay(FIXTURES / 'velocity_feedback.jsonl')
        self.assertTrue(all(r['active'] and r['allowed'] for r in rows))
        self.assertNotEqual(rows[0]['q'], rows[-1]['q'])
        self.assertLess(rows[-1]['acceleration_target'][0], rows[0]['acceleration_target'][0])

    def test_command_timeout_is_independent_of_new_state_samples(self):
        records = [json.loads(s) for s in (FIXTURES / 'velocity_feedback.jsonl').read_text().splitlines()]
        initial = records[1]
        records = [records[0], initial]
        for seq in range(2, 10):
            stamp = 1_000_000_000 + seq * 50_000_000
            state = dict(initial['state'], sequence=seq, stamp_ns=stamp)
            records.append(dict(kind='tick', dt=.05, ros_now_ns=stamp, state=state))
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'commands_lost.jsonl'
            path.write_text('\n'.join(json.dumps(r) for r in records) + '\n')
            rows = self.run_replay(path)
        self.assertTrue(rows[0]['active'])
        self.assertGreater(rows[-1]['state_sequence'], rows[0]['state_sequence'])
        self.assertEqual(rows[-1]['command_sequence'], 1)
        self.assertIn(rows[-1]['phase'], ('DESCENT', 'RECOVERY_DESCENT'))
        self.assertFalse(rows[-1]['active'])

    def test_duplicate_state_sequence_does_not_renew_receiver_freshness(self):
        records = [json.loads(s) for s in (FIXTURES / 'velocity_feedback.jsonl').read_text().splitlines()]
        initial = records[1]
        records = [records[0], initial]
        for seq in range(2, 10):
            stamp = 1_000_000_000 + seq * 50_000_000
            command = dict(initial['command'], sequence=seq, stamp_ns=stamp)
            records.append(dict(kind='tick', dt=.05, ros_now_ns=stamp, state=initial['state'], command=command))
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'state_duplicates.jsonl'
            path.write_text('\n'.join(json.dumps(r) for r in records) + '\n')
            rows = self.run_replay(path)
        self.assertTrue(rows[0]['active'])
        self.assertEqual(rows[-1]['state_sequence'], 1)
        self.assertGreater(rows[-1]['command_sequence'], rows[0]['command_sequence'])
        self.assertIn(rows[-1]['phase'], ('DESCENT', 'RECOVERY_DESCENT'))
        self.assertFalse(rows[-1]['active'])

    def test_invalid_mode_type_is_reported_as_an_input_error(self):
        records = [json.loads(s) for s in (FIXTURES / 'follow_land.jsonl').read_text().splitlines()]
        records[1]['command']['mode'] = []
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'bad_mode.jsonl'
            path.write_text('\n'.join(json.dumps(r) for r in records) + '\n')
            result = self.run_replay(path, success=False)
        self.assertIn('replay input:', result.stderr)
        self.assertNotIn('Traceback', result.stderr)

    def test_invalid_record_is_rejected_before_host_driver(self):
        for value in ('{"schema_version":2,"kind":"session","dataset_kind":"synthetic"}\n',
                      '{"schema_version":1,"kind":"session","dataset_kind":"synthetic"}\n{"kind":"tick","state":{"position":[1,2]}}\n',
                      '{"schema_version":1,"kind":"session","dataset_kind":"synthetic"}\n{"kind":"tick","dt":NaN}\n'):
            with self.subTest(value=value), tempfile.TemporaryDirectory() as td:
                path = Path(td) / 'invalid.jsonl'; path.write_text(value)
                result = self.run_replay(path, success=False)
                self.assertIn('replay input:', result.stderr)


if __name__ == '__main__':
    unittest.main()
