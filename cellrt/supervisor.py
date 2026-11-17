"""The independent supervisor. It reads; it commands nothing except hold and 'recalibrate'.

Independence, as far as software can show it:
  - it imports neither the runtime nor the policy (a test checks the import list);
  - its per-cycle verdict uses the fixture's binary seat sensor and the gripper, never the camera the
    policy aims with (a camera as noisy as the placement cannot check the placement);
  - the seat sensor is four corner gates (`seat[0..3]`), each tested every cycle by its
    own transition, at three moments: all four empty before
    the load; all four closed with the tool down and the vacuum still on; all four closed again once
    the tool is away. If they do not close with the tool down, the supervisor refuses the release for
    that cycle: the part comes back up on the cup and goes to the reject place, so a failed insertion
    leaves the nest empty. The read with the tool away catches a part that left with the tool or
    rose behind it. A gate stuck high is found at the first pre-load
    read; a gate stuck low after three placements that the other three call good. The pattern tells a gate from a part: one corner high before a load is a gate
    (a part left in the nest presses at least two); the same single corner low three times running
    is a gate (a tilt does not pick the same corner three times);
  - what a bad gate does is a per-job setting, `on_gate_fault`: "stop" (default: hold and ask for the
    gate to be swapped) or "flag" (mask that one gate, raise a notice, keep running on the other
    three; a second bad gate stops the cell);
  - it has its own heartbeat to the runtime, and the runtime refuses motion without it.
The camera's verify reading is used only as a trend (an exploratory drift monitor): an alarm asks for a recalibration between cycles; it stops nothing.

In a product this logic belongs on a separate small controller with its own power and its own wire
to the motor enable. Here it is a separate object, and `tools/hang_probe.py` runs it as a separate
process to time what it does when the runtime stops answering.
"""
from __future__ import annotations

from dataclasses import dataclass, field

# reply reasons that are not the cell's or the policy's failure and never flag a cycle on their own
QUIET = ("replay", "unauthorized", "malformed")


def _bits(seat):
    """Four gates as a string for the cycle record: '1' down, '0' up, '-' no reading."""
    return "".join("-" if b is None else "1" if b else "0" for b in seat)


@dataclass
class Verdict:
    cycle: int
    flag: bool
    reasons: list = field(default_factory=list)
    hold: str = ""
    recalibrate: bool = False
    notices: list = field(default_factory=list)   # maintenance notices: the cycle is not flagged by these
    interlock: bool = False                       # the release was refused for this cycle; the cell was not stopped


class Trend:
    """EWMA of (verify reading - baseline); baseline = mean of the first `base_n` readings after a
    calibration."""

    def __init__(self, h_mm=1.3629, lam=0.1, base_n=50, centre_mm=0.8):
        self.h, self.lam, self.base_n, self.centre_mm = h_mm, lam, base_n, centre_mm
        self.reset()

    def reset(self):
        self.n, self.sum, self.z, self.alarm = 0, [0.0, 0.0], [0.0, 0.0], False

    def add(self, xy) -> bool:
        if xy is None:
            return False
        self.n += 1
        if self.n <= self.base_n:
            self.sum[0] += xy[0]
            self.sum[1] += xy[1]
            return False
        b = (self.sum[0] / self.base_n, self.sum[1] / self.base_n)
        self.z = [(1 - self.lam) * self.z[i] + self.lam * (xy[i] - b[i]) for i in (0, 1)]
        # Added to the plain monitor: the reading is part-to-socket offset, so after a good calibration its
        # mean must sit near zero. A baseline that is itself off-centre (a biased policy version, a
        # calibration that did not fit) would otherwise be learned as normal.
        off_centre = self.n == self.base_n + 1 and (b[0] ** 2 + b[1] ** 2) ** 0.5 > self.centre_mm
        self.alarm = off_centre or (self.z[0] ** 2 + self.z[1] ** 2) ** 0.5 > self.h
        return self.alarm


GATE_LOW_STREAK = 3      # same single corner low after release, the other three down, this many cycles running


class Supervisor:
    def __init__(self, tap, log, trend: Trend | None = None, hold_line=None, heartbeat_line=None, precheck=True,
                 on_gate_fault="stop", interlock=True, away_read=True):
        if on_gate_fault not in ("stop", "flag"):
            raise ValueError("on_gate_fault is 'stop' or 'flag'")
        self.tap, self.log, self.precheck, self.on_gate_fault = tap, log, precheck, on_gate_fault
        self.interlock, self.away_read = interlock, away_read   # ablation switches; both on in a product
        self.trend = trend or Trend()
        self._hold = hold_line or (lambda reason, cycle=None: None)   # the one wire that can stop motion
        self._beat = heartbeat_line or (lambda now: None)
        self.alive = True                                     # fault hook: the supervisor process dies
        self.cur = None
        self.streak = {}                                      # consecutive flagged cycles by reason
        self.masked = {}                                      # corner -> "high" | "low" | "missing": gates no longer believed
        self.bad_gate = None                                  # (corner, how) named by the last gate verdict
        self._low_corner, self._low_n = None, 0

    def heartbeat(self, now_ms: int):
        if self.alive:
            self._beat(now_ms)

    def gate_swapped(self):
        """A person replaced a gate (or the whole sensor): believe all four again."""
        self.masked, self.bad_gate, self._low_corner, self._low_n = {}, None, None, 0

    def _gate_fault(self, corner: int, how: str):
        """One gate is wrong. Stop and name it, or (per-job setting) mask it and keep running on three."""
        self.bad_gate = (corner, how)
        if self.on_gate_fault == "flag" and not self.masked:
            self.masked[corner] = how
            self.cur.notices.append(f"seat_gate_masked:{corner}:{how}")
            if how == "low" and self.cur.reasons == ["not_seated"]:
                self.cur.reasons, self.cur.flag = [], False   # the other three corners are down: this part is seated
            return False
        self._flag("seat_gate_stuck" if how != "missing" else "seat_gate_missing", hold=True)   # one named gate either way
        return True

    def begin_cycle(self, cycle: int, now_ms: int):
        self.cur = Verdict(cycle, False)
        self._done = set()
        pre = self.tap.seat()
        self.log.append(now_ms, "sense", cycle=cycle, at="pre", seat=_bits(pre))
        live = [i for i in range(len(pre)) if i not in self.masked]
        none = [i for i in live if pre[i] is None]
        high = [i for i in live if pre[i]]
        if none:
            if len(none) == 1:
                self._gate_fault(none[0], "missing")
            else:
                self._flag("seat_missing", hold=True)         # more than one gate silent: connector or supply
        elif high and self.precheck:
            if len(high) == 1 and len(live) == len(pre):
                self._gate_fault(high[0], "high")             # a part left in the nest presses at least two corners
            else:
                self._flag("seat_precheck", hold=True)        # nest not empty, or the whole sensor reads high
        return self.cur

    def on_reply(self, rep, now_ms: int):
        """Called with every runtime reply."""
        v = self.cur
        if rep.outcome != "done":
            if rep.reason not in QUIET and rep.reason != "supervisor_hold":
                self._flag(rep.reason, hold=rep.reason in ("tracking_error", "torque", "stop_input"))
            return
        self._done.add(rep.step)
        if rep.step == 0:
            g = self.tap.gripper_holds()
            self.log.append(now_ms, "sense", cycle=v.cycle, at="grasp", gripper=g)
            if g is None:
                self._flag("gripper_missing", hold=True)
            elif not g:
                self._flag("grasp_empty")
        elif rep.step == 1:
            xy = self.tap.verify_obs_mm()
            self.log.append(now_ms, "sense", cycle=v.cycle, at="place", verify=xy)
            if self.trend.add(xy):
                v.recalibrate = True
            if self._read_seat("down", now_ms) and self.interlock and not v.hold:
                v.interlock = True
                self._hold("not_seated", v.cycle)             # this release only: the part goes back up on the cup
        elif rep.step == 2 and not self.away_read:
            self._read_seat("release", now_ms)                # ablation: the last read is taken with the tool still down
        elif rep.step == 3 and self.away_read:
            self._read_seat("away", now_ms)

    def _read_seat(self, at: str, now_ms: int) -> bool:
        """One read of the four gates where all should be closed. True when a live gate is open."""
        v = self.cur
        s = self.tap.seat()
        self.log.append(now_ms, "sense", cycle=v.cycle, at=at, seat=_bits(s))
        live = [i for i in range(len(s)) if i not in self.masked]
        none = [i for i in live if s[i] is None]
        low = [i for i in live if s[i] is False]
        if len(none) > 1:
            self._flag("seat_missing", hold=True)             # more than one gate silent: connector or supply
        elif none and self._gate_fault(none[0], "missing"):
            pass                                              # stopped, gate named
        elif low:
            self._flag("not_seated" if at != "away" or "not_seated" in v.reasons else "not_seated_after_release")
            if at == "away":
                return True
            # One corner up with the other three down is a tilt, or that corner's gate. The same
            # corner three cycles running is the gate.
            one = low[0] if len(low) == 1 and not self.masked else None
            self._low_n = (self._low_n + 1 if one == self._low_corner else 1) if one is not None else 0
            self._low_corner = one
            if self._low_n >= GATE_LOW_STREAK:
                self._low_corner, self._low_n = None, 0
                self._gate_fault(one, "low")
            return "not_seated" in v.reasons                  # False if the gate was just masked: the part is seated
        elif at != "away":
            self._low_corner, self._low_n = None, 0
        return False

    def end_cycle(self, now_ms: int) -> Verdict:
        v = self.cur
        if not {2, 3 if self.away_read else 2} <= self._done and not v.flag:
            self._flag("incomplete")                          # no release, or no read with the tool away, and nobody said why
        for r in list(self.streak):
            if r not in v.reasons:
                self.streak[r] = 0
        for r in v.reasons:
            self.streak[r] = self.streak.get(r, 0) + 1
        self.log.append(now_ms, "verdict", cycle=v.cycle, flag=v.flag, reasons=v.reasons, hold=v.hold,
                        recalibrate=v.recalibrate, notices=v.notices)
        return v

    def calibrated(self):
        """Tell the supervisor a new calibration is in force: the trend baseline starts again."""
        self.trend.reset()

    def _flag(self, reason: str, hold=False):
        v = self.cur
        v.flag = True
        if reason not in v.reasons:
            v.reasons.append(reason)
        if hold and not v.hold:
            v.hold = reason
            self._hold(reason)                                # at once: the runtime ends the cycle in the safe state

    def runtime_silent(self):
        """The runtime stopped answering: open the enable line."""
        self._flag("runtime_silent", hold=True)
