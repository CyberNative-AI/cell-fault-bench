"""The runtime: the only component that commands motion.

It trusts nothing it is sent. Every request is authenticated, parsed strictly, checked against the
manifest in force, the cycle sequence, the age of its observation, the cell map and the envelope,
and only then executed. A refusal is a normal reply. A refusal from an authenticated policy inside
an open cycle ends that cycle in the safe state (held part to the reject bin, arm home). Bytes that
fail authentication or parsing never change state, so garbage on the wire cannot abort good work.

Software limits are not engineered limits. A physical cell needs an independent stop.
"""
from __future__ import annotations

from dataclasses import dataclass

from .contracts import Malformed, RuntimeReply, open_envelope, parse_request
from .plant import DST_NEST, SRC_NEST


@dataclass(frozen=True)
class Limits:
    env_lo: tuple = (100.0, -120.0, 0.0)     # mm, the box the tool may be sent into
    env_hi: tuple = (280.0, 120.0, 120.0)
    nest_tol_mm: float = 2.5                 # a grasp or place target must be this close to its nest in the cell map
    torque_max: float = 1.5                  # N·m peak reported by the drive
    step_timeout_ms: int = 8000              # an open cycle with no accepted step for this long is aborted
    sup_timeout_ms: int = 150                # no supervisor heartbeat for this long: no motion. The value the process
    #                                          probe was timed at (tools/hang_probe.py); the tool's spring travel is
    #                                          chosen with it
    # When is an observation too old to act on? A per-job setting.
    #   "event": it must have been taken after the last thing that changed what it looks at (a part
    #            arrived at the source; a part was released into, or cleared from, the destination;
    #            anyone or anything recovered the cell). Its age alone does not refuse it, up to
    #            obs_max_age_ms. For a fixtured cell, where nothing moves between events.
    #   "clock": age at the start of motion must not exceed the request's own deadline_ms (the cell model's
    #            interface v0). For a scene that can change with no event the runtime hears of.
    freshness: str = "event"
    obs_max_age_ms: int = 300_000            # backstop in "event" mode (a choice, not a derived number)


NO_STATE_CHANGE = ("unauthorized", "malformed", "replay")   # refusals that are not the policy's own act


def admit_static(body, manifest, lim: Limits = Limits()) -> str:
    """The checks that need no cell and no clock, for the shadow-replay gate: '' or a refusal reason."""
    try:
        req = parse_request(body)
    except Malformed:
        return "malformed"
    if (req.policy_id, req.policy_version) != (manifest.policy_id, manifest.policy_version):
        return "unknown_policy"
    if req.calibration_id != manifest.calibration_id:
        return "stale_calibration"
    if req.kind in ("grasp", "place"):
        x = req.target_xyz_mm
        if any(not lo <= v <= hi for v, lo, hi in zip(x, lim.env_lo, lim.env_hi)):
            return "envelope"
        nest = SRC_NEST if req.kind == "grasp" else DST_NEST
        if ((x[0] - nest[0]) ** 2 + (x[1] - nest[1]) ** 2) ** 0.5 > lim.nest_tol_mm:
            return "not_a_nest"
        if req.expected_error_mm_p99 > req.capture_radius_mm:
            return "error_exceeds_capture"
    return ""


class Runtime:
    def __init__(self, cell, manifest, keys: dict, log, limits: Limits = Limits()):
        self.cell, self.manifest, self.keys, self.log, self.lim = cell, manifest, keys, log, limits
        self.last_seq = {}
        self.open_cycle = None
        self.next_step = 0
        self.last_closed = -1
        self.last_progress_ms = 0
        self.last_grasp_frame = -1
        self.scene_ms = {"src": 0, "dst": 0}  # when each nest's contents last changed, as far as the runtime knows
        self.hold_reason = ""
        self.hold_cycle = None               # a hold for one cycle only (the release interlock) ends with that cycle
        self.stop_input = False
        self.sup_seen_ms = 0
        self.hung = False                    # fault hook: the process stops answering
        self.aborts = []                     # (cycle, reason) for the driver and the supervisor
        self.counts = {}

    # ---------------------------------------------------------------- lines from outside
    def supervisor_heartbeat(self, now_ms: int):
        self.sup_seen_ms = now_ms

    def scene_event(self, where: str, now_ms: int):
        """A line from outside: something changed at the source nest, the destination nest, or the
        whole cell (a person's hands, a recovery). In a product: the part-present gate's edge, the
        seat gates' edges, the stop input's release. Logged, so replay sees the same events."""
        for k in (("src", "dst") if where == "cell" else (where,)):
            self.scene_ms[k] = max(self.scene_ms[k], now_ms)
        self.log.append(now_ms, "scene", where=where)

    def set_hold(self, reason: str, cycle=None):
        """The supervisor's line. With `cycle`, the hold ends when that cycle has been put in the safe
        state: the supervisor refuses this release, it does not stop the cell."""
        self.hold_reason, self.hold_cycle = reason, cycle

    def clear_hold(self):
        self.hold_reason, self.hold_cycle = "", None

    def load_manifest(self, manifest):
        """Called by the release manager between cycles only."""
        if self.open_cycle is not None:
            raise RuntimeError("manifest change inside a cycle")
        self.manifest = manifest
        self.log.append(self.last_progress_ms, "manifest", **manifest.public())

    # ---------------------------------------------------------------- one envelope
    def submit(self, env, now_ms: int) -> RuntimeReply:
        if self.hung:
            raise TimeoutError("runtime does not answer")
        if not self.log.append(now_ms, "rx", env=env if isinstance(env, dict) else repr(env)[:200]):
            return self._refuse(None, now_ms, "log_unavailable", abort=True, logit=False)
        if self.stop_input:
            return self._refuse(None, now_ms, "stop_input", abort=True)
        if self.hold_reason:
            return self._refuse(None, now_ms, "supervisor_hold", self.hold_reason, abort=True)
        if now_ms - self.sup_seen_ms > self.lim.sup_timeout_ms:
            return self._refuse(None, now_ms, "no_supervisor", abort=True)
        try:
            seq, body = open_envelope(self.keys, env)
        except PermissionError:
            return self._refuse(None, now_ms, "unauthorized")
        except Malformed as e:
            return self._refuse(None, now_ms, "malformed", str(e))
        if seq <= self.last_seq.get(env["key_id"], -1):
            return self._refuse(None, now_ms, "replay")
        self.last_seq[env["key_id"]] = seq
        try:
            req = parse_request(body)
        except Malformed as e:
            return self._refuse(None, now_ms, "malformed", str(e))
        m = self.manifest
        if (req.policy_id, req.policy_version) != (m.policy_id, m.policy_version):
            return self._refuse(req, now_ms, "unknown_policy", abort=True)
        if req.calibration_id != m.calibration_id:
            return self._refuse(req, now_ms, "stale_calibration", abort=True)
        if self.open_cycle is None:
            if req.step != 0 or req.cycle <= self.last_closed:
                return self._refuse(req, now_ms, "out_of_sequence")
        elif req.cycle != self.open_cycle or req.step != self.next_step:
            return self._refuse(req, now_ms, "out_of_sequence", abort=True)
        if req.kind in ("grasp", "place"):
            if req.kind == "grasp" and req.frame_id <= self.last_grasp_frame:
                return self._refuse(req, now_ms, "frozen_frame", abort=True)
            age = now_ms - req.observed_at_ms
            if age < 0:                      # an observation from the future passes any age test: refuse it
                return self._refuse(req, now_ms, "stale_observation", "observed_at is after now", abort=True)
            if self.lim.freshness == "clock":
                if age > req.deadline_ms:
                    return self._refuse(req, now_ms, "stale_observation", "older than deadline_ms", abort=True)
            elif req.observed_at_ms < self.scene_ms["src" if req.kind == "grasp" else "dst"]:
                return self._refuse(req, now_ms, "stale_observation", "taken before the nest last changed", abort=True)
            elif age > self.lim.obs_max_age_ms:
                return self._refuse(req, now_ms, "stale_observation", "older than obs_max_age_ms", abort=True)
            x = req.target_xyz_mm
            if any(not lo <= v <= hi for v, lo, hi in zip(x, self.lim.env_lo, self.lim.env_hi)):
                return self._refuse(req, now_ms, "envelope", abort=True)
            nest = SRC_NEST if req.kind == "grasp" else DST_NEST
            if ((x[0] - nest[0]) ** 2 + (x[1] - nest[1]) ** 2) ** 0.5 > self.lim.nest_tol_mm:
                return self._refuse(req, now_ms, "not_a_nest", abort=True)
            if req.expected_error_mm_p99 > req.capture_radius_mm:
                return self._refuse(req, now_ms, "error_exceeds_capture", abort=True)
        # ------------------------------------------------------------ execute
        if req.step == 0:
            self.open_cycle, self.next_step = req.cycle, 0
            self.last_grasp_frame = req.frame_id
        r = self.cell.do(req.kind, req.target_xyz_mm[:2])
        if not r["moved"]:
            return self._refuse(req, now_ms, "tracking_error", abort=True, outcome="stopped")
        if r["torque_peak"] > self.lim.torque_max:
            return self._refuse(req, now_ms, "torque", abort=True, outcome="stopped", torque=r["torque_peak"])
        self.next_step += 1
        self.last_progress_ms = now_ms + r["duration_ms"]
        if req.kind in ("grasp", "release"):  # the runtime's own acts change a nest: grasp empties the source
            self.scene_ms["src" if req.kind == "grasp" else "dst"] = self.last_progress_ms
        if req.kind == "home":
            self.last_closed, self.open_cycle = req.cycle, None
        rep = RuntimeReply(req.cycle, req.step, "done", duration_ms=r["duration_ms"], torque_peak=round(r["torque_peak"], 3))
        self._log_reply(now_ms, rep)
        return rep

    # ---------------------------------------------------------------- watchdog
    def tick(self, now_ms: int):
        """Abort an open cycle that has made no progress. Returns the abort reply or None."""
        if self.open_cycle is not None and now_ms - self.last_progress_ms > self.lim.step_timeout_ms:
            rep = RuntimeReply(self.open_cycle, self.next_step, "timed_out", "no_heartbeat",
                               duration_ms=self._abort(self.open_cycle, "no_heartbeat"))
            self._log_reply(now_ms, rep)
            return rep
        return None

    # ---------------------------------------------------------------- internals
    def _abort(self, cycle, reason) -> int:
        """Safe state: whatever is held goes to the reject bin and the arm goes home.
        If the arm cannot move the cycle is closed and the supervisor's hold decides what follows."""
        self.aborts.append((cycle, reason))
        self.last_closed = max(self.last_closed, cycle if cycle is not None else self.last_closed)
        self.open_cycle = None
        if self.hold_cycle is not None and cycle == self.hold_cycle:
            self.hold_reason, self.hold_cycle = "", None
        r = self.cell.do("reject")
        return r["duration_ms"]              # the nests are untouched by the reject move: no scene event

    def _refuse(self, req, now_ms, reason, detail="", abort=False, outcome="refused", torque=0.0, logit=True):
        self.counts[reason] = self.counts.get(reason, 0) + 1
        cyc = req.cycle if req is not None else (self.open_cycle if self.open_cycle is not None else -1)
        dur = 0
        if abort and reason not in NO_STATE_CHANGE and self.open_cycle is not None:
            dur = self._abort(self.open_cycle, reason)
        elif abort and req is not None and req.step == 0 and req.cycle > self.last_closed:
            self.last_closed = req.cycle          # a refused first step closes that cycle number
        rep = RuntimeReply(cyc, req.step if req is not None else -1, outcome, reason, detail[:120], dur, torque)
        if logit:
            self._log_reply(now_ms, rep)
        return rep

    def _log_reply(self, now_ms, rep: RuntimeReply):
        self.log.append(now_ms, "reply", cycle=rep.cycle, step=rep.step, outcome=rep.outcome, reason=rep.reason,
                        duration_ms=rep.duration_ms)
