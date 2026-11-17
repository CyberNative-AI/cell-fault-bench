"""The bench against a device that is not the example cell: examples/toy_cell.py, no arm model."""
import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)

from cellrt import faults as F  # noqa: E402
from cellrt.line import Line  # noqa: E402
from examples.toy_cell import ToyCell  # noqa: E402


class ToyDevice(unittest.TestCase):
    def test_clean_cycles_seat_and_none_is_stopped(self):
        L = Line(2000, device=ToyCell, ladder=False)
        for _ in range(300):
            L.run_cycle()
            L.rt.clear_hold()
        seated = sum(r["seated"] for r in L.records)
        self.assertGreater(seated, 250)                   # the toy's 1.2 mm error seats most parts at 3 mm
        self.assertFalse(any(r["flag"] for r in L.records if r["seated"]))
        self.assertTrue(all(r["flag"] for r in L.records if not r["seated"]))
        self.assertIsNone(L.phys)

    def test_seat_stuck_high_is_flagged_on_the_first_cycle(self):
        L = Line(2001, device=ToyCell)
        for _ in range(20):
            L.run_cycle()
        f = L.inject(F.SeatStuck(how="high"))
        rec = L.run_cycle()
        self.assertTrue(rec["flag"])
        self.assertIn("seat_precheck", rec["reasons"])
        self.assertIsNotNone(f.first_flag_ms)

    def test_campaign_on_the_toy_runs_what_it_supports_and_lists_the_rest(self):
        with tempfile.TemporaryDirectory() as out:
            env = dict(os.environ, OMP_NUM_THREADS="1")
            p = subprocess.run([sys.executable, "-m", "cellrt.campaign", "--quick", "--procs", "2", "--device",
                                "examples.toy_cell:ToyCell", "--out", out], cwd=ROOT, env=env,
                               capture_output=True, text=True, timeout=1200)
            self.assertEqual(p.returncode, 0, p.stderr[-2000:])
            d = json.load(open(os.path.join(out, "campaign.json")))
        self.assertEqual(d["meta"]["cell"]["device"], "examples.toy_cell:ToyCell")
        not_run = set(d["meta"]["not_run"])
        self.assertTrue({"base_shift_1mm", "lash_growth_to_2deg", "camera_noise_1mm", "three_gates"} <= not_run)
        self.assertIn("seat_sensor_high", d["persistent"])
        self.assertIn("corrupted_packet", d["transient"])
        silent = sum(v["on_hit_cycles"]["E"] + v["on_other_cycles"]["E"] for v in d["transient"].values())
        silent += sum(v["counts"]["E"] for v in d["persistent"].values())
        self.assertEqual(silent, 0)
        self.assertIn("Device examples.toy_cell:ToyCell", p.stdout)
        self.assertIn("Not run on this device", p.stdout)


if __name__ == "__main__":
    unittest.main()
