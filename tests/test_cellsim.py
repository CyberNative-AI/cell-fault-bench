"""Model checks for the example cell: arm kinematics and the lash model."""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from cellsim import so101  # noqa: E402
from cellsim.cell import DEG, Mech, Unit, sample_targets  # noqa: E402


class Kinematics(unittest.TestCase):
    def setUp(self):
        self.arm = so101.load_arm()
        self.rng = np.random.default_rng(1)

    def test_moving_mass_matches_model(self):
        # five link bodies + moving jaw from the MJCF inertials
        self.assertAlmostEqual(self.arm.meta["moving_mass"], 0.100006 + 0.103 + 0.104 + 0.079 + 0.087 + 0.012, 6)

    def test_zero_pose_reaches_forward(self):
        p, a = self.arm.fk(np.zeros((1, 5)))
        self.assertGreater(p[0, 0], 0.35)           # tool in front of the base
        self.assertAlmostEqual(float(a[0, 0]), 1.0, 2)  # approach axis along +x at the zero pose

    def test_ik_round_trip(self):
        tgt = sample_targets(200, self.rng)
        q, ok = self.arm.ik_down(tgt)
        self.assertTrue(ok.all())
        p, a = self.arm.fk(q)
        self.assertLess(np.abs(p - tgt).max(), 5e-5)
        self.assertLess(np.rad2deg(np.arccos(-a[:, 2])).max(), 0.1)

    def test_gravity_torque_is_minus_potential_gradient(self):
        arm = self.arm
        q = self.rng.uniform(-0.8, 0.8, (20, 5))
        payload = 0.02

        def U(qq):
            origins, rots = arm.frames(qq)
            u = np.zeros(len(qq))
            for i, l in enumerate(arm.links):
                com = origins[i] + np.einsum("nij,j->ni", rots[i], l.com)
                u += l.mass * so101.G * com[:, 2]
            tip = origins[-1] + np.einsum("nij,j->ni", rots[-1], arm.tool_pos)
            return u + (arm.extra_tip_mass + payload) * so101.G * tip[:, 2]

        tau = arm.gravity_torque(q, payload)
        for j in range(5):
            d = np.zeros(5)
            d[j] = 1e-6
            num = -(U(q + d) - U(q - d)) / 2e-6
            np.testing.assert_allclose(tau[:, j], num, atol=1e-6)


class Mechanism(unittest.TestCase):
    def test_lash_stays_inside_gap_and_play_is_hysteretic(self):
        rng = np.random.default_rng(2)
        for model in ("play", "random"):
            u = Unit(0, Mech(lash_model=model, lash_jitter=0.0))
            q, _ = u.nominal.ik_down(sample_targets(50, rng))
            y0 = u.settle(q, rng, None)
            # motor-side equilibrium: remove lash by rebuilding it with zero lash
            u0 = Unit(0, Mech(lash_model=model, lash_deg=0.0))
            base = u0.settle(q, rng, None)
            self.assertLessEqual(np.abs(y0 - base).max(), 0.5 * DEG + 1e-12)
        u = Unit(0, Mech(lash_model="play", lash_jitter=0.0))
        q, _ = u.nominal.ik_down(sample_targets(50, rng))
        y = u.settle(q, rng, None)
        # a command change smaller than what is left of the gap must not move an unloaded link (pan)
        y2 = u.settle(q, rng, y)
        np.testing.assert_allclose(y2[:, 0], y[:, 0], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
