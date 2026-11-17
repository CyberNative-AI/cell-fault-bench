"""The smallest device the bench runs against, written without the example cell's arm model.

Copy it as the start of an adapter for your own cell simulation (README, "Run the bench against
your cell"). Here the placement error is a round Gaussian; yours comes from your simulator.

    OMP_NUM_THREADS=1 python3 -m cellrt.campaign --device examples.toy_cell:ToyCell --out results/toy

What a device is, method by method:
  begin_cycle()          a new part waits at the source nest; the last one has left the destination,
                         unless `blocked` (a part lying across the nest) is set
  do(kind, target_xy)    the only way to move: "grasp", "place" (lower into the nest, vacuum on),
                         "release", "home", and "reject" (carry what is held to the reject bin).
                         Returns dict(moved, torque_peak, duration_ms). Targets are task-frame mm,
                         the nests at cellrt.plant.SRC_NEST and DST_NEST.
  tap()                  the supervisor's read-only wires: seat() four corner gates (True = that
                         corner of a part is down, None = no reading), gripper_holds(), and
                         verify_obs_mm() (one overhead look before release, or None)
  truth()                dict(seated=..., miss_mm=...): what really happened. Only the bench reads it
  twin()                 an independent copy at the start of a cycle, for the blind replay
  recalibrate()          automatic recalibration; may do nothing
  hands_clear()          a person clears a part lying across the nest
  faults                 a cellrt.plant.DeviceFaults; the bench sets its fields, the device honours
                         the ones it lists in SUPPORTS (the others' fault classes are not run)
  c                      the nest's lead-in, mm
"""
import copy

import numpy as np

from cellrt.plant import DST_NEST, SRC_NEST, DeviceFaults

CORNERS = 4


class ToyCell:
    SUPPORTS = frozenset({"seat_stuck", "seat_corner", "seat_p_miss", "seat_p_false", "gripper_stuck", "stall",
                          "torque_spike", "blocked"})

    def __init__(self, unit_seed, c_mm=3.0, seed=0, sd_mm=1.2):
        self.c, self.sd = c_mm, sd_mm
        self.faults = DeviceFaults()
        self.rng = np.random.default_rng([unit_seed, seed, 5])
        self.blocked = self.holding = False
        self.begin_cycle()

    def begin_cycle(self):
        self.nest_occupied = self.blocked
        self.at, self.offset, self.ok = "home", None, None
        self.tool_down = self.released = self.attempted = False
        self.lifted = (0, 1) if self.blocked else None
        self.err = self.rng.normal(0.0, self.sd, 2)          # this cycle's placement error, mm
        self.cam = self.rng.normal(0.0, 0.7, 2)               # this cycle's camera error, mm
        self.noise = self.rng.random()                        # one draw for sensor misreads and which corner lifts

    def do(self, kind, target_xy=None):
        f = self.faults
        if f.stall:
            return dict(moved=False, torque_peak=0.2, duration_ms=0)
        torque = 2.5 if f.torque_spike else 0.3
        if kind == "grasp":
            self.holding = bool(np.linalg.norm(np.subtract(target_xy, SRC_NEST[:2])) <= 6.0)
            self.at = "src"
        elif kind == "place":
            self.offset = self.err + np.subtract(target_xy, DST_NEST[:2])
            self.at, self.tool_down = "dst", True
            if self.holding:
                self.ok = bool(not self.nest_occupied and np.linalg.norm(self.offset) <= self.c)
                if not self.ok and not self.nest_occupied:
                    k = int(self.noise * 1e6) % CORNERS
                    self.lifted = (k, (k + 1) % CORNERS)      # hung on the lead-in: one edge up
        elif kind == "release":
            if self.holding and self.at == "dst":
                self.attempted = True
                self.nest_occupied = self.nest_occupied or bool(self.ok)
            self.holding, self.released = False, True
        elif kind == "home":
            self.at, self.tool_down = "home", False
        elif kind == "reject":
            if self.holding and self.tool_down and self.ok is False and not self.nest_occupied:
                self.lifted = None                            # the failed part comes away on the cup
            self.holding, self.at, self.tool_down = False, "home", False
        else:
            raise ValueError(kind)
        return dict(moved=True, torque_peak=torque, duration_ms=500 if kind == "release" else 2000)

    def tap(self):
        return Tap(self)

    def truth(self):
        seated = bool(self.nest_occupied and not self.blocked and self.released)
        return dict(seated=seated, miss_mm=None if self.offset is None else float(np.linalg.norm(self.offset)))

    def twin(self):
        nc = copy.copy(self)
        nc.rng = copy.deepcopy(self.rng)
        return nc

    def recalibrate(self):
        pass

    def hands_clear(self):
        self.blocked = self.nest_occupied = False
        self.lifted = None

    def _seat(self):
        f = self.faults
        if f.seat_stuck == "missing":
            return [None] * CORNERS
        if f.seat_stuck in ("high", "low"):
            return [f.seat_stuck == "high"] * CORNERS
        pressed = self.tool_down and self.holding and self.at == "dst"
        if (self.nest_occupied and not self.blocked) or (pressed and self.ok):
            bits = [True] * CORNERS
            if self.noise < f.seat_p_false:
                bits[int(self.noise * 1e6) % CORNERS] = False
        elif self.lifted is not None and (pressed or self.attempted or self.blocked):
            bits = [self.noise < f.seat_p_miss or i not in self.lifted for i in range(CORNERS)]
        else:
            bits = [False] * CORNERS
        for i, how in f.seat_corner.items():
            bits[i] = None if how == "missing" else how == "high"
        return bits


class Tap:
    """Read-only: no route to do()."""

    def __init__(self, cell):
        self._c = cell

    def seat(self):
        return self._c._seat()

    def gripper_holds(self):
        g = self._c.faults.gripper_stuck
        return None if g == "missing" else (g == "high") if g else self._c.holding

    def verify_obs_mm(self):
        c = self._c
        return None if c.offset is None else (c.offset + c.cam).tolist()
