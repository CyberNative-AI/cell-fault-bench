"""One simulated cell at work: policy client -> wire -> runtime -> devices, the supervisor beside
it, a recovery ladder after every flagged cycle. Virtual time in milliseconds.

Every cycle is also run "blind": the same bytes given to a controller with no authentication, no
checks and no seat sensor, from the same physical starting state. That gives the counterfactual
the detection definition needs (README, Counting): a stopped cycle that would have been
good is a false alarm, not a caught failure.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import math
import time

import numpy as np

from .contracts import KINDS
from .eventlog import EventLog
from .plant import Physics, SimCell
from .policy import ScriptedPolicy
from .release import Manifest
from .runtime import Limits, Runtime
from .supervisor import QUIET, Supervisor

KEY, KID = b"slice-device-key-not-a-secret", "policy-1"
SUP_SEES_SILENT_RUNTIME_MS = 150    # the supervisor's own timeout on the runtime (process probe: 143 ms median at 150)

# Time each rung of the ladder takes. Tags: CALC from the cell model's move counts and timings (
# 1.0 s a move, 0.1 s a look), ESTIMATE for everything that restarts software or involves a person.
RUNG_MS = dict(
    retry=0,                 # the next cycle is the retry; the reject move is already in the cycle time
    resync=2_000,            # ESTIMATE: policy re-reads the manifest
    rotate_log=1_000,        # ESTIMATE
    restart_policy=15_000,   # ESTIMATE: process restart and camera pipeline warm-up
    restart_supervisor=10_000,  # ESTIMATE
    restart_runtime=15_000,  # ESTIMATE: restart, journal read, held part to the reject bin
    recalibrate=30_000,      # CALC: 12 poses x (move + one-direction approach + look) + fit; about half a minute
    rollback=45_000,         # ESTIMATE: pointer switch, restart, self-test
    assist=90_000,           # ESTIMATE: bundle upload, one model call, answer shown to the buyer
    hands=180_000,           # ESTIMATE: the buyer walks over and does one named thing (assumed: 3 min)
)
HANDS_FOR = {  # what a remote agent can ask a person to do, by what the supervisor reported
    "seat_precheck": ["clear_nest", "swap_seat_sensor"],
    "seat_gate_stuck": ["swap_seat_sensor"],
    "seat_gate_missing": ["swap_seat_sensor"],
    "seat_missing": ["swap_seat_sensor"],
    "gripper_missing": ["swap_gripper"],
    "tracking_error": ["free_arm"],
    "torque": ["free_arm"],
    "not_seated": ["swap_seat_sensor", "swap_joint_module", "clean_camera", "clear_nest"],
    "not_seated_after_release": ["swap_gripper", "clear_nest"],
    "grasp_empty": ["swap_gripper", "swap_joint_module", "clean_camera"],
    "frozen_frame": ["reseat_camera"],
    "stale_observation": ["reseat_camera"],
}
POLICY_REASONS = {"envelope", "not_a_nest", "error_exceeds_capture", "unknown_policy"}
LINK_REASONS = {"stale_observation", "frozen_frame", "incomplete", "no_heartbeat", "out_of_sequence"}
GATE_NAMED = {"seat_gate_stuck", "seat_gate_missing"}   # the supervisor has named one gate (sup.bad_gate): no model is asked
HOLD_REASONS = {"seat_precheck", "seat_gate_stuck", "seat_gate_missing", "seat_missing", "gripper_missing", "tracking_error", "torque", "stop_input"}
GROUPS = dict(link=LINK_REASONS, policy=POLICY_REASONS, seat={"not_seated", "not_seated_after_release", "grasp_empty"})


class Fault:
    """Base class. A transient fault hits each cycle with probability p; a persistent one from its
    onset until the rung or hands action named in `clears` is applied."""
    name, persistent, p, clears, hands = "fault", False, 1.0, (), None

    def __init__(self, **kw):
        self.__dict__.update(kw)
        self.hit = False
        self.onset_ms = self.first_flag_ms = self.cleared_ms = None
        self.silent_before_flag = 0
        self.rungs = []

    def start(self, line):
        pass

    def stop(self, line):
        pass

    def before_cycle(self, line):
        self.hit = self.persistent or line.rng.random() < self.p

    def requests(self, line, reqs):
        return reqs

    def channel(self, line, envs):
        return envs


# The cell a campaign runs on, when a Line is built without naming one: lead-in (mm) and arm
# ("model", "p50", "p90"; cellsim/arms.py). campaign.main sets it in every worker process.
CELL = dict(c_mm=3.0, arm="model", device=None)


def set_cell(c_mm=3.0, arm="model", device=None):
    CELL.update(c_mm=float(c_mm), arm=arm, device=device)


def load_device(spec):
    """'module:factory' -> the factory. factory(unit_seed, c_mm, seed) returns a device (README, "Run the
    bench against your cell"); the factory's SUPPORTS lists the device fault fields it honours."""
    import importlib
    mod, _, attr = spec.partition(":")
    return getattr(importlib.import_module(mod), attr)


def supports(spec):
    """Device fault fields the device honours; a device that lists none gets no device faults."""
    return SimCell.SUPPORTS if spec is None else frozenset(getattr(load_device(spec), "SUPPORTS", ()))


class Line:
    def __init__(self, unit_seed: int, c_mm=None, stack="calibrated_uni", lash_deg=0.5, seed=0, log_path=None,
                 assist_accuracy=0.9, ladder=True, on_gate_fault="stop", freshness="event", tilt_share=None,
                 interlock=True, away_read=True, arm=None, device=None):
        c_mm = CELL["c_mm"] if c_mm is None else c_mm
        arm = CELL["arm"] if arm is None else arm
        device = CELL["device"] if device is None else device
        if device is None:                         # the example cell; stack, lash_deg, arm, tilt_share are its settings
            self.phys = Physics(unit_seed, stack, lash_deg, stream=seed, arm=arm)
            self.cell = SimCell(self.phys, c_mm, seed, tilt_share)
        else:                                      # your device: a factory, or "module:factory"
            make = load_device(device) if isinstance(device, str) else device
            self.cell = make(unit_seed, c_mm, seed)
            self.phys = getattr(self.cell, "phys", None)
        self.calibrations = 1
        self.log = EventLog(log_path)
        self.manifest = Manifest(1, "transfer", "1.0.0", "cal-1", "cellrt-0.1", "cfg-0")
        self.previous = None                       # (manifest, policy settings) of the other slot
        self.rt = Runtime(self.cell, self.manifest, {KID: KEY}, self.log, Limits(freshness=freshness))
        self.sup = Supervisor(self.cell.tap(), self.log, hold_line=self.rt.set_hold,
                              heartbeat_line=self.rt.supervisor_heartbeat, on_gate_fault=on_gate_fault,
                              interlock=interlock, away_read=away_read)
        self.policy = ScriptedPolicy(KEY, KID, "transfer", "1.0.0", "cal-1", c_mm)
        self.rng = np.random.default_rng([unit_seed, seed, 31337])
        self.now = self.cycle = self.frame = 0
        self.faults, self.records, self.recoveries = [], [], []
        self.assist_accuracy, self.ladder = assist_accuracy, ladder
        self.last_recal_cycle = -10 ** 9
        self.recal_cycles = []
        self.tried_hands, self.suspect = [], None
        self.gstreak = dict(link=0, policy=0, seat=0)
        self.verify_hist = []
        self.canary_until, self.canary_trips = -1, 0
        self.cpu_ns = 0
        self.notices = []
        self.idle_ms = self.look_early_ms = 0
        self.log.append(0, "manifest", **self.manifest.public())

    # ------------------------------------------------------------------ faults
    def inject(self, fault: Fault):
        fault.onset_ms = self.now
        fault.start(self)
        self.faults.append(fault)
        return fault

    def _clear(self, action: str):
        for f in list(self.faults):
            if f.persistent and action in f.clears:
                f.stop(self)
                f.cleared_ms = self.now
                self.faults.remove(f)

    # ------------------------------------------------------------------ one cycle
    def run_cycle(self) -> dict:
        cyc, t0 = self.cycle, self.now
        self.cycle += 1
        self.frame += 1
        self.cell.begin_cycle()
        self.idle_ms = self.look_early_ms = 0
        for f in self.faults:
            f.before_cycle(self)
        # A part arrives at the source nest: the event the look must come after. Faults may make the
        # look early (taken before the part arrived) or leave the cell idle between look and go.
        self.rt.scene_event("src", self.now)
        reqs = self.policy.requests(cyc, max(0, self.now - self.look_early_ms), self.frame) if self.policy.alive else []
        self.now += self.idle_ms
        self.sup.heartbeat(self.now)
        for f in self.faults:
            if f.hit:
                reqs = f.requests(self, reqs)
        envs = [self._seal(r) for r in reqs]
        for f in self.faults:
            if f.hit:
                envs = f.channel(self, envs)
        good_blind = self._blind(envs)
        c0 = time.perf_counter_ns()
        v = self.sup.begin_cycle(cyc, self.now)
        if not v.hold:
            hung = False
            if not envs:
                self.now += self.rt.lim.step_timeout_ms           # nothing arrived: the cycle watchdog runs out
            for env in envs:
                self.now += 10
                self.sup.heartbeat(self.now)
                try:
                    rep = self.rt.submit(env, self.now)
                except TimeoutError:
                    hung = True
                    break
                self.sup.on_reply(rep, self.now)
                self.now += rep.duration_ms
                if rep.outcome != "done" and rep.reason not in QUIET:
                    break
            if hung:
                self.now += SUP_SEES_SILENT_RUNTIME_MS
                self.sup.runtime_silent()                         # the supervisor opens the enable line
            elif self.rt.open_cycle is not None:
                self.now += self.rt.lim.step_timeout_ms
                self.sup.heartbeat(self.now)
                rep = self.rt.tick(self.now)
                if rep:
                    self.sup.on_reply(rep, self.now)
                    self.now += rep.duration_ms
        verdict = self.sup.end_cycle(self.now)
        self.cpu_ns += time.perf_counter_ns() - c0
        R = set(verdict.reasons)
        for g, members in GROUPS.items():
            self.gstreak[g] = self.gstreak.get(g, 0) + 1 if R & members else 0
        xy = self.cell.tap().verify_obs_mm()
        self.verify_hist = (self.verify_hist + [None if xy is None else (xy[0] ** 2 + xy[1] ** 2) ** 0.5])[-3:]
        if self.cycle <= self.canary_until and (verdict.flag or verdict.recalibrate):
            self.canary_trips += 1
        seated = self.cell.truth()["seated"]
        rec = dict(cycle=cyc, t0=t0, t1=self.now, good_blind=good_blind, seated=seated, flag=verdict.flag,
                   reasons=list(verdict.reasons), hold=verdict.hold, faults=[f.name for f in self.faults if f.hit],
                   notices=list(verdict.notices), masked=sorted(self.sup.masked), interlock=verdict.interlock)
        self.records.append(rec)
        self.notices += [(cyc, n) for n in verdict.notices]
        for f in self.faults:
            if f.persistent and f.first_flag_ms is None:
                if verdict.flag or verdict.recalibrate or verdict.notices:
                    f.first_flag_ms = self.now
                elif not seated:
                    f.silent_before_flag += 1
        if self.ladder and (verdict.flag or verdict.recalibrate):
            self.recover(verdict)
        return rec

    def _seal(self, item) -> dict:
        """Requests are sealed in order. A str is a raw body from a policy with a bug: it still holds the key."""
        if isinstance(item, str):
            self.policy.seq += 1
            mac = hmac.new(KEY, f"{KID}|{self.policy.seq}|{item}".encode(), hashlib.sha256).hexdigest()
            return {"key_id": KID, "seq": self.policy.seq, "body": item, "mac": mac}
        return self.policy.seal(item)

    def _blind(self, envs) -> bool:
        """The same bytes through a controller with no checks: would the part have ended seated?"""
        if self.rt.hung or self.cell.faults.stall or self.cell.faults.torque_spike:
            return False
        nc = self.cell.twin()
        for env in envs:
            try:
                b = json.loads(env["body"])
                kind, x = b["kind"], b["target_xyz_mm"]
                xy = (float(x[0]), float(x[1]))
                if kind not in KINDS or not all(math.isfinite(t) for t in xy):
                    continue
            except Exception:
                continue
            nc.do(kind, xy)
        return nc.truth()["seated"]

    # ------------------------------------------------------------------ recovery ladder
    def decide(self, v) -> str:
        R, s = set(v.reasons), self.sup.streak
        if self.previous and self.cycle <= self.canary_until and (self.canary_trips >= 2 or v.recalibrate):
            return "rollback"                      # canary: a new version that trips twice, or moves the trend, goes back
        if "runtime_silent" in R:
            return "restart_runtime"
        if "no_supervisor" in R:
            return "restart_supervisor"
        if "log_unavailable" in R:
            return "rotate_log"
        if R & HOLD_REASONS:
            return "assist"
        if "stale_calibration" in R:
            return ("resync", "restart_policy", "assist")[min(s.get("stale_calibration", 1), 3) - 1]
        if R & POLICY_REASONS:
            n = self.gstreak["policy"]
            return "retry" if n < 2 else ("rollback" if self.previous else "restart_policy") if n == 2 else "assist"
        if R & LINK_REASONS:
            n = self.gstreak["link"]
            return "retry" if n < 2 else "restart_policy" if n == 2 else "assist"
        if R & GROUPS["seat"]:
            n = self.gstreak["seat"]
            recent = sum(1 for r in self.records[-60:] if set(r["reasons"]) & GROUPS["seat"])
            if n < 3 and recent < 4:
                return "retry"                     # one-off miss: the next cycle is the retry
            if "not_seated" in R and all(x is not None and x < 0.5 * self.cell.c for x in self.verify_hist):
                self.suspect = ["clear_nest", "swap_seat_sensor"]   # the camera says the placements were centred
                return "assist"
            # three in a row, or four in the last sixty cycles: something changed. Recalibrate once; if
            # that was done recently and it is still failing, a person or a part is needed.
            return "recalibrate" if self.cycle - self.last_recal_cycle > 60 else "assist"
        if v.recalibrate:
            # Trend or centre alarm with no failed cycle. The automatic calibration is itself noisy (about
            # one fit in ten leaves the centre off), so a second and a third try are cheap and usually
            # enough; three recalibrations inside 200 cycles that did not quiet it means help is needed.
            return "recalibrate" if sum(1 for c in self.recal_cycles if self.cycle - c <= 200) < 3 else "assist"
        return "retry"

    def recover(self, v):
        rung = self.decide(v)
        t0, hands, action = self.now, False, rung
        self.now += RUNG_MS[rung]
        if rung == "resync":
            self._sync_policy()
        elif rung == "restart_policy":
            self.policy.alive, self.policy.frozen_frame = True, None
            self._sync_policy()
        elif rung == "restart_runtime":
            self.rt.hung = False
            self.rt.open_cycle = None
            self.cell.do("reject")                 # journal says a part may be held: clear it before resuming
        elif rung == "restart_supervisor":
            self.sup.alive = True
        elif rung == "rotate_log":
            self.log.capacity, self.log.failed = None, False
        elif rung == "recalibrate":
            self.recalibrate()
        elif rung == "rollback":
            self.rollback()
        elif rung == "assist":
            action, hands = self._assist(v)
        self._clear(action)
        self._clear(rung)
        if action == "swap_seat_sensor":
            self.sup.gate_swapped()
        self.rt.clear_hold()
        if hands or rung in ("restart_runtime", "recalibrate"):
            self.rt.scene_event("cell", self.now)   # hands or the arm were in the cell: every earlier look is void
        self.sup.heartbeat(self.now)
        ev = dict(t=t0, cycle=v.cycle, reasons=list(v.reasons), rung=rung, action=action, hands=hands, ms=self.now - t0,
                  named_by=("supervisor" if GATE_NAMED & set(v.reasons) else "agent") if hands else "")
        self.recoveries.append(ev)
        self.log.append(self.now, "recovery", **ev)
        for f in self.faults:
            if f.persistent:
                f.rungs.append(action)

    def _assist(self, v):
        """A remote agent reads the bundle (reasons, streaks, sensor readings, verify trend, last
        update) and names one action. It can order a rollback itself; anything physical it can only
        ask the buyer to do. `assist_accuracy` is the chance it names the right physical action first."""
        if self.previous and (set(v.reasons) & POLICY_REASONS or any(f.name.startswith("bad_update") for f in self.faults)):
            self.now += RUNG_MS["rollback"]
            self.rollback()
            return "rollback", False
        cands = []
        for r in v.reasons:
            cands += [a for a in HANDS_FOR.get(r, []) if a not in cands]
        if v.recalibrate and not cands:
            cands = ["swap_joint_module", "clean_camera"]
        if not cands:
            return "resume", False
        if GATE_NAMED & set(v.reasons):
            # The four bits name the part to change (which gate, stuck which way: sup.bad_gate). No
            # model is asked and nothing is guessed.
            self.tried_hands.append("swap_seat_sensor")
            self.now += RUNG_MS["hands"]
            return "swap_seat_sensor", True
        if self.suspect:
            cands, self.suspect = [a for a in self.suspect if a in cands] or cands, None
        need = [f.hands for f in self.faults if f.persistent and f.hands in cands]
        left = [a for a in cands if a not in self.tried_hands] or cands
        if need and need[0] in left and self.rng.random() < self.assist_accuracy:
            pick = need[0]
        else:
            pick = left[int(self.rng.integers(len(left)))]
        self.tried_hands.append(pick)
        self.now += RUNG_MS["hands"]
        if pick == "clear_nest":
            self.cell.hands_clear()
        return pick, True

    def _sync_policy(self):
        m = self.rt.manifest
        self.policy.calibration_id, self.policy.policy_id, self.policy.policy_version = \
            m.calibration_id, m.policy_id, m.policy_version

    def recalibrate(self):
        self.cell.recalibrate()
        self.calibrations += 1
        self.last_recal_cycle = self.cycle
        self.recal_cycles.append(self.cycle)
        m = self.rt.manifest.with_calibration(f"cal-{self.calibrations}")
        self.rt.load_manifest(m)
        self._sync_policy()
        self.sup.calibrated()

    # ------------------------------------------------------------------ update and rollback (in-memory form of release.Slots)
    def update(self, version: str, bias_mm=(0.0, 0.0), scale=1.0, canary=50):
        self.previous = (self.rt.manifest, self.policy.bias_mm, self.policy.scale)
        m = Manifest(self.rt.manifest.release + 1, "transfer", version, self.rt.manifest.calibration_id,
                     "cellrt-0.1", "cfg-0")
        self.rt.load_manifest(m)
        self.policy.bias_mm, self.policy.scale = bias_mm, scale
        self._sync_policy()
        self.canary_until, self.canary_trips = self.cycle + canary, 0

    def rollback(self):
        if not self.previous:
            return
        m, bias, scale = self.previous
        self.rt.load_manifest(m.with_calibration(self.rt.manifest.calibration_id))
        self.policy.bias_mm, self.policy.scale = bias, scale
        self._sync_policy()
        self.previous = None
