"""Simulated devices: an arm in a fixtured cell, a gripper, four seat gates at the corners of the
destination nest (`seat[0..3]`), a stop input, and a camera clock.

The physical outcome of every cycle comes from the cell model (`cellsim`): unit variation, servo sag, gear lash, calibration, placement error. This file
adds only what the software under test sees: what the devices report, and how they misreport.

Simulation of a stated model. No device was moved; nothing here is a physical measurement.
"""
from __future__ import annotations

import copy
import os
import sys
from dataclasses import dataclass, field

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from cellsim.arms import mech_for  # noqa: E402
from cellsim.cell import (Cam, Mech, Unit, calibrate, ctrl_calibrated, ctrl_servo, home_state,  # noqa: E402
                          sample_targets)

SRC_NEST = (190.0, -60.0, 30.0)      # mm, task frame: the cell map (two nests)
DST_NEST = (190.0, 60.0, 30.0)
REJECT_BIN = (150.0, 0.0, 60.0)
GRASP_CAPTURE_MM = 6.0               # the cell model's task defaults
CARRY = 0.5
TILT_MAX_DEG = 4.0
CORNERS = 4                          # seat gates, numbered round the part


class Physics:
    """One unit's placement errors, cycle after cycle, from cellsim. Unit seeds >= 2000 only:
    lower seeds were used to fit and freeze the cell model."""

    def __init__(self, unit_seed: int, stack="calibrated_uni", lash_deg=0.5, cam_sd=0.5, batch=128, stream=0, arm=None):
        if unit_seed < 2000:
            raise ValueError("unit seeds below 2000 were used to fit the cell model")
        self.stack, self.cam_sd, self.batch = stack, cam_sd, batch
        self.unit = Unit(unit_seed, mech_for(arm, Mech(lash_deg=lash_deg)))
        self.rng = np.random.default_rng([unit_seed, 4242, stream])
        self.cal = calibrate(self.unit, Cam(cam_sd), self.rng, k=12, uni=(stack == "calibrated_uni"))
        self.calibrations = 1
        self._rows = []

    def _place(self):
        cam = Cam(self.cam_sd)
        if self.stack == "servo_cal":
            return lambda T, r, y: ctrl_servo(self.unit, T, cam, r, y, self.cal)
        return lambda T, r, y: ctrl_calibrated(self.unit, T, cam, r, y, self.cal)

    def _fill(self):
        n, rng, place = self.batch, self.rng, self._place()
        src, dst = sample_targets(n, rng), sample_targets(n, rng)
        a = place(src, rng, home_state(self.unit, n, rng))
        b = place(dst, rng, a["y"])
        e1 = (a["p"] - src)[:, :2] * 1e3
        e2 = (b["p"] - dst)[:, :2] * 1e3
        tilt = np.rad2deg(b["tilt"])
        noise = rng.normal(0, self.cam_sd * np.sqrt(2.0), (n, 2))
        self._rows = [dict(e1=e1[i], e2=e2[i], tilt_ok=bool(tilt[i] <= TILT_MAX_DEG), valid_a=bool(a["valid"][i]),
                           valid_b=bool(b["valid"][i]), cam_noise=noise[i]) for i in range(n)]

    def draw(self) -> dict:
        if not self._rows:
            self._fill()
        return self._rows.pop()

    # four mechanical and camera faults, applied to the unit as built
    def apply_fault(self, name: str, value: float):
        if name == "base_shift_mm":
            self.unit.true.base_pos = self.unit.true.base_pos + np.array([0.0, value * 1e-3, 0.0])
        elif name == "lash_deg":
            self.unit = self.unit.with_mech(lash_deg=value)
        elif name == "offset_drift_deg":
            self.unit.offsets = self.unit.offsets + np.deg2rad(value)
        elif name == "cam_sd_mm":
            self.cam_sd = value
        else:
            raise ValueError(name)
        self._rows = []

    def recalibrate(self):
        """Automatic recalibration of the unit as it now is (12 looks). Fixes what calibration can."""
        self.cal = calibrate(self.unit, Cam(self.cam_sd), self.rng, k=12, uni=(self.stack == "calibrated_uni"))
        self.calibrations += 1
        self._rows = []


@dataclass
class DeviceFaults:
    seat_stuck: str | None = None     # all four gates (connector, supply): "high", "low", "missing" (no reading)
    seat_corner: dict = field(default_factory=dict)   # one gate: {corner: "high" | "low" | "missing"}
    seat_p_miss: float = 0.0          # all four read seated when the part is not seated (lift below the gates' acceptance)
    seat_p_false: float = 0.0         # one gate reads empty when the part is seated
    gripper_stuck: str | None = None  # "high", "low", "missing"
    stall: bool = False               # the arm does not move; reported joints stay where they were
    torque_spike: bool = False        # a move reports a torque peak above the limit (collision, binding)
    block_p: float = 0.0              # a failed insertion that is released leaves the part across the nest; hands needed
    wedge_p: float = 0.0              # a failed insertion stays in the nest even when the tool lifts with the vacuum on
    cling_p: float = 0.0              # a seated part leaves with the tool, or rises when it lifts


class SimCell:
    """Devices behind the runtime. `do` is the only way to move; sensors are read through `tap()` by
    the supervisor, which gets no way to call `do`.

    The example cell's implementation of the device interface (README, "Run the bench against your
    cell"; `examples/toy_cell.py` is a smaller one written without the cell model)."""

    SUPPORTS = frozenset({"seat_stuck", "seat_corner", "seat_p_miss", "seat_p_false", "gripper_stuck", "stall",
                          "torque_spike", "blocked", "block_p", "wedge_p", "cling_p", "physics"})

    def __init__(self, physics: Physics, c_mm=3.0, seed=0, tilt_share=None):
        self.phys, self.c = physics, c_mm
        # How a part that did not seat lies. None: from the cell model (one corner up only when the
        # tool tilt is over the limit, which at 1 degree of lash or less never happens; otherwise one
        # edge up). A number: that share of bad seats lift one corner only, whatever the cause; the
        # stated fixture mix is 0.3 tilted to 0.6 hung, so 1/3.
        # Neither is measured.
        self.tilt_share = tilt_share
        self.faults = DeviceFaults()
        self.rng = np.random.default_rng([physics.unit.seed, seed, 77])
        self.blocked = False
        self._blocked_lift = (0, 1)
        self.holding = False
        self.nest_occupied = False
        self.begin_cycle()

    def begin_cycle(self):
        """A new part is at the source nest; the previous one has been taken away downstream
        (unless the nest is blocked by a part that failed to seat)."""
        self.row = self.phys.draw()
        self.nest_occupied = self.blocked
        self.at, self.offset, self.inhand = "home", None, np.zeros(2)
        self.released = self.attempted = False
        self.tool_down = self.clung = False
        self.down_ok = None
        self.seat_noise = self.rng.random()
        self.seat_corner_draw = int(self.seat_noise * 1e6) % CORNERS   # which corner or edge a bad seat lifts (no extra draw)
        self.lifted = self._blocked_lift if self.blocked else None
        self.lay = None                  # truth only: how this cycle's bad seat lay under the tool, kept after the part is lifted away

    # ------------------------------------------------------------------ motion
    def do(self, kind: str, target_xy=None) -> dict:
        f = self.faults
        if f.stall:
            return dict(moved=False, torque_peak=0.2, duration_ms=0)
        torque = 2.5 if f.torque_spike else float(0.25 + 0.1 * self.rng.random())
        r = self.row
        if kind == "grasp":
            d = np.asarray(target_xy) - np.array(SRC_NEST[:2])
            miss = r["e1"] + d
            self.holding = bool(r["valid_a"] and np.linalg.norm(miss) <= GRASP_CAPTURE_MM)
            self.inhand = CARRY * miss
            self.at = "src"
            dur = 2600
        elif kind == "place":
            d = np.asarray(target_xy) - np.array(DST_NEST[:2])
            # Sign kept from the model this bench was tuned on: physically the part rides opposite its grasp
            # miss, so `- self.inhand`. In a paired study of the cell model alone (carry 0.5, no fault bench),
            # a base shift showed in the verify reading at about 1.45x its size with `+` and about 0.48x with
            # `-`, so a base_shift label stands for a bump roughly three times larger (README, Limits). The
            # campaign was run with `+` only; no campaign result is shown for the physical sign.
            self.offset = r["e2"] + d + self.inhand
            self.at = "dst"
            self.tool_down = True
            if self.holding:
                # The tool is down with the vacuum on: the part is pressed into the nest, or onto whatever
                # stops it. How it lies when it does not seat, in the fixture's terms:
                # tilted = one corner up, the other three down; hung on the lead-in = one edge up, so two
                # neighbouring corners up. A part already lying across the nest keeps its own pattern.
                self.down_ok = bool(r["valid_b"] and r["tilt_ok"] and not self.nest_occupied
                                    and float(np.linalg.norm(self.offset)) <= self.c)
                if not self.down_ok and not self.nest_occupied:
                    k = self.seat_corner_draw
                    if self.tilt_share is None:
                        one = bool(r["valid_b"] and not r["tilt_ok"] and float(np.linalg.norm(self.offset)) <= self.c)
                    else:
                        one = (self.seat_noise * 1e3) % 1.0 < self.tilt_share
                    self.lifted = (k,) if one else (k, (k + 1) % CORNERS)
                    self.lay = self.lifted
            dur = 2600
        elif kind == "release":
            if self.holding:
                self.attempted = self.at == "dst"
                if self.at == "dst" and self.down_ok:
                    self.nest_occupied = True
                    self.clung = bool(f.cling_p) and self.rng.random() < f.cling_p
                elif self.at == "dst" and self.rng.random() < f.block_p:
                    self.blocked, self._blocked_lift = True, self.lifted
                self.holding = False
            self.released = True
            dur = 500
        elif kind == "home":
            if self.clung:               # the part went up with the tool, or sprang up behind it: not in the nest
                self.nest_occupied, self.clung, self.lifted, self.attempted = False, False, None, False
            self.at, self.tool_down, dur = "home", False, 600
        elif kind == "reject":       # recovery: carry whatever is held to the reject bin, open, go home
            if self.holding and self.tool_down and self.down_ok is False and not self.nest_occupied:
                # lifted off a failed insertion with the vacuum still on: the part comes away, unless it is wedged
                if bool(f.wedge_p) and self.rng.random() < f.wedge_p:
                    self.blocked, self._blocked_lift = True, self.lifted
                else:
                    self.lifted = None
            self.holding, self.at, self.tool_down, dur = False, "home", False, 3000
        else:
            raise ValueError(kind)
        return dict(moved=True, torque_peak=torque, duration_ms=dur)

    def twin(self):
        """An independent copy at the start of a cycle, for the blind replay: the same bytes through a
        controller with no checks. Moving the twin must not move this cell or use its random stream."""
        nc = copy.copy(self)
        nc.rng = copy.deepcopy(self.rng)
        nc.inhand = np.zeros(2)
        return nc

    def recalibrate(self):
        """Automatic recalibration of the arm as it now is."""
        self.phys.recalibrate()

    def hands_clear(self):
        """A person clears the cell: removes a part lying across the nest."""
        self.blocked = False
        self.nest_occupied = False
        self.lifted = None

    # ------------------------------------------------------------------ truth (never read by runtime or supervisor)
    def truth(self) -> dict:
        seated = bool(self.nest_occupied and not self.blocked and self.released)
        return dict(seated=seated, miss_mm=None if self.offset is None else round(float(np.linalg.norm(self.offset)), 3))

    # ------------------------------------------------------------------ sensors
    def _seat(self):
        """Four gates, one a corner: True = that corner of a part is down, None = no reading.
        Seated part, or a part pressed home by the tool: all four down. Bad seat: the lifted corners
        read up, the rest down, unless the lift is under the gates' acceptance height (`seat_p_miss`:
        all four read down). Empty nest or a part dropped on the way: all up."""
        f = self.faults
        if f.seat_stuck == "missing":
            return [None] * CORNERS
        if f.seat_stuck in ("high", "low"):
            return [f.seat_stuck == "high"] * CORNERS
        pressed = self.tool_down and self.holding and self.at == "dst"      # tool down, vacuum on
        seated = bool(self.nest_occupied and not self.blocked) or (pressed and bool(self.down_ok))
        if seated:
            bits = [True] * CORNERS
            if self.seat_noise < f.seat_p_false:
                bits[self.seat_corner_draw] = False
        elif self.lifted is not None and (pressed or self.attempted or self.blocked):
            miss = self.seat_noise < f.seat_p_miss
            bits = [miss or i not in self.lifted for i in range(CORNERS)]
        else:
            bits = [False] * CORNERS
        for i, how in f.seat_corner.items():
            bits[i] = None if how == "missing" else how == "high"
        return bits

    def _grip(self):
        f = self.faults
        if f.gripper_stuck == "missing":
            return None
        if f.gripper_stuck in ("high", "low"):
            return f.gripper_stuck == "high"
        return self.holding

    def tap(self):
        return SensorTap(self)


class SensorTap:
    """Read-only view for the supervisor: its own wires to the sensors, no route to the motors."""

    def __init__(self, cell: SimCell):
        self._c = cell

    def seat(self):
        """[bit, bit, bit, bit] for the four corner gates; an entry is None when that gate gives no reading."""
        return self._c._seat()

    def gripper_holds(self):
        return self._c._grip()

    def verify_obs_mm(self):
        """One overhead look at part and socket before release (the policy's own verify look). Too noisy to judge one
        cycle; used only as a trend."""
        c = self._c
        if c.offset is None:
            return None
        return (c.offset + c.row["cam_noise"]).tolist()
