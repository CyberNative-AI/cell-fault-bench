"""Replay: re-run the runtime's decisions from a log and compare with what the log says it decided.

    python3 -m cellrt.replay <log.jsonl>

What it proves: given the envelopes received, their times, the manifests in force and the lines the
supervisor held, the runtime's code produces the same replies. It is how a remote reader finds out
why a cycle was refused without visiting the machine, and how a new runtime version is checked
against a unit's own history before it is installed. It replays decisions, not physics: what the
devices answered is taken from the log, never re-simulated.
"""
from __future__ import annotations

import json
import sys

from .eventlog import EventLog, verify
from .release import Manifest
from .runtime import Limits, Runtime


class _LoggedCell:
    """Stands in for the devices: answers each motion with what the log recorded."""

    def __init__(self):
        self.next_answer = None
        self.faults = None

    def do(self, kind, target_xy=None):
        return self.next_answer or dict(moved=True, torque_peak=0.3, duration_ms=0)


def replay(lines, keys: dict, limits: Limits = Limits()) -> dict:
    ok, bad, _ = verify(lines)
    ev = [json.loads(x) for x in lines]
    cell = _LoggedCell()
    rt = None
    mismatches, decisions = [], 0
    for i, e in enumerate(ev):
        if e["k"] == "manifest":
            m = Manifest(**e["d"])
            if rt is None:
                rt = Runtime(cell, m, keys, EventLog(), limits)
            else:
                rt.manifest = m
        elif e["k"] == "scene" and rt is not None:
            for k in (("src", "dst") if e["d"]["where"] == "cell" else (e["d"]["where"],)):
                rt.scene_ms[k] = max(rt.scene_ms[k], e["t"])
        elif e["k"] == "recovery" and e["d"]["rung"] == "restart_runtime":
            rt.open_cycle = None
        elif e["k"] == "rx":
            # the logged reply is the next "reply" event; heartbeat, hold and stop lines are inputs
            j = next((k for k in range(i + 1, min(i + 4, len(ev))) if ev[k]["k"] in ("reply", "rx")), None)
            if j is None or ev[j]["k"] != "reply":
                continue
            want = ev[j]["d"]
            rt.hold_reason = "hold" if want["reason"] == "supervisor_hold" else ""
            rt.sup_seen_ms = e["t"] if want["reason"] != "no_supervisor" else -10 ** 9
            rt.stop_input = want["reason"] == "stop_input"
            cell.next_answer = dict(moved=want["reason"] != "tracking_error",
                                    torque_peak=9.9 if want["reason"] == "torque" else 0.3,
                                    duration_ms=want["duration_ms"])
            got = rt.submit(e["d"]["env"], e["t"])
            decisions += 1
            if (got.outcome, got.reason, got.cycle, got.step) != (want["outcome"], want["reason"], want["cycle"], want["step"]):
                mismatches.append(dict(line=i, t=e["t"], logged=[want["outcome"], want["reason"]],
                                       replayed=[got.outcome, got.reason]))
        elif e["k"] == "reply" and e["d"]["outcome"] == "timed_out" and rt is not None:
            rt.tick(e["t"])
    return dict(chain_ok=ok, first_bad_line=bad, events=len(ev), decisions=decisions, mismatches=mismatches[:20],
                mismatch_count=len(mismatches))


if __name__ == "__main__":
    from .line import KEY, KID
    out = replay([x.rstrip("\n") for x in open(sys.argv[1])], {KID: KEY})
    print(json.dumps(out, indent=1))
    sys.exit(0 if out["chain_ok"] and not out["mismatch_count"] else 1)
