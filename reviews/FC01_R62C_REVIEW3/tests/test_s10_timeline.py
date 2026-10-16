import importlib.util
import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('s10_timeline', ROOT/'tools'/'s10_timeline.py')
timeline = importlib.util.module_from_spec(spec)
spec.loader.exec_module(timeline)

class S10TimelineTest(unittest.TestCase):
    def test_source_log_detection_and_no_fabricated_handoff(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'fixture.jsonl'
            rows = [
                {'kind':'state','phase':'THRUST_RAMP','seq':1,'heartbeat_enabled':True,'armed':True,'offboard':True},
                {'kind':'state','phase':'DESCENT','seq':2,'heartbeat_enabled':True,'armed':True,'offboard':True},
                {'kind':'state','phase':'SAFETY','seq':3,'heartbeat_enabled':False,'armed':True,
                 'offboard':True,'reason':'DESCENT_TIMEOUT_ARMED_OFFBOARD_PX4_HANDOFF_UNVERIFIED'},
            ]
            p.write_text('\n'.join(json.dumps(x) for x in rows)+'\n', encoding='utf-8')
            out=io.StringIO()
            with redirect_stdout(out):self.assertEqual(timeline.summarize(p),0)
            self.assertIn('ARMED_OFFBOARD_HEARTBEAT_OFF_OBSERVATIONS=1',out.getvalue())
            self.assertIn('DESCENT_TIMEOUT',out.getvalue())

if __name__ == '__main__':unittest.main()
