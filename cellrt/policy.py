"""Policy-side client for the slice: one fixtured transfer per cycle, four requests.

This stands where a learned or scripted policy stands. It holds no motor access; it builds requests and
seals them. The knobs are the model errors the campaign injects.
"""
from __future__ import annotations

from .contracts import KINDS, seal
from .plant import DST_NEST, SRC_NEST


class ScriptedPolicy:
    def __init__(self, key: bytes, key_id: str, policy_id: str, policy_version: str, calibration_id: str,
                 c_mm=3.0, p99_mm=1.5, deadline_ms=10_000):
        self.key, self.key_id = key, key_id
        self.policy_id, self.policy_version, self.calibration_id = policy_id, policy_version, calibration_id
        self.c, self.p99, self.deadline_ms = c_mm, p99_mm, deadline_ms
        self.seq = 0
        self.bias_mm = (0.0, 0.0)      # model error: every place target off by this much
        self.scale = 1.0               # model error: unit mix-up (metres sent as millimetres = 0.001)
        self.alive = True
        self.frozen_frame = None       # camera pipeline delivers one old frame for ever

    def requests(self, cycle: int, observed_at_ms: int, frame_id: int) -> list:
        if self.frozen_frame is not None:
            observed_at_ms, frame_id = self.frozen_frame
        out = []
        for step, kind in enumerate(KINDS):
            nest = SRC_NEST if kind in ("grasp", "home") else DST_NEST
            x = [nest[0] * self.scale, nest[1] * self.scale, nest[2] * self.scale]
            if kind == "place":
                x[0] += self.bias_mm[0]
                x[1] += self.bias_mm[1]
            out.append(dict(cycle=cycle, step=step, kind=kind, target_xyz_mm=x, approach="uni",
                            capture_radius_mm=self.c, expected_error_mm_p99=self.p99,
                            calibration_id=self.calibration_id, policy_id=self.policy_id,
                            policy_version=self.policy_version, deadline_ms=self.deadline_ms,
                            observed_at_ms=observed_at_ms, frame_id=frame_id))
        return out

    def seal(self, body: dict) -> dict:
        self.seq += 1
        return seal(self.key, self.key_id, self.seq, body)
