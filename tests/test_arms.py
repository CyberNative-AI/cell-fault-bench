"""Measured-arm option of the cell model (cellsim/arms.py): the default arm is unchanged, a measured
arm keeps each joint inside its own play plus settle scatter, and a lash fault scales its play."""
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..")]

from cellsim import arms, cell  # noqa: E402
from cellrt.line import CELL, Line, set_cell  # noqa: E402


class Arms(unittest.TestCase):
    def test_model_arm_is_the_default_mech(self):
        self.assertEqual(arms.mech_for("model"), cell.Mech())
        self.assertEqual(arms.mech_for(None, cell.Mech(lash_deg=1.0)), cell.Mech(lash_deg=1.0))

    def test_measured_play_and_scatter_bound_each_joint(self):
        m = arms.mech_for("p90")
        u = cell.Unit(2000, m)
        rng = np.random.default_rng(0)
        q = np.zeros((20000, 5))
        y = u._lash(q, np.zeros_like(q), rng, np.zeros_like(q) + 1.0)   # far above: every joint pinned at +B
        dev = np.rad2deg(y - q)
        B = np.asarray(m.play_deg)
        S = np.asarray(m.scatter_deg)
        np.testing.assert_allclose(dev.mean(0), B, atol=0.03)
        np.testing.assert_allclose(dev.std(0), S, rtol=0.05)

    def test_lash_fault_scales_measured_play(self):
        u = cell.Unit(2000, arms.mech_for("p50"))
        np.testing.assert_allclose(cell.half_width_rad(u.with_mech(lash_deg=1.0).mech),
                                   2 * cell.half_width_rad(u.mech))

    def test_line_takes_the_campaign_cell(self):
        try:
            set_cell(5.0, "p50")
            L = Line(2000)
            self.assertEqual(L.cell.c, 5.0)
            self.assertEqual(L.phys.unit.mech.play_deg, arms.ARMS["p50"]["play_deg"])
        finally:
            set_cell()
        self.assertEqual(CELL, dict(c_mm=3.0, arm="model", device=None))
        self.assertIsNone(Line(2000).phys.unit.mech.play_deg)


if __name__ == "__main__":
    unittest.main()
