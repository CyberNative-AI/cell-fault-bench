"""The scorecard: what the checks caught, what they stopped that was good, and what got through, on the
same cycles. Reads the campaign's output; prints a Markdown table.

    python3 -m cellrt.scorecard results/campaign.json             # one row per fault family
    python3 -m cellrt.scorecard results/campaign.json --by-class  # one row per fault class

`python3 -m cellrt.campaign` prints both tables itself when it finishes: the fault families, then the
seat-sensor and cling cases (a sensor that misreads, a part that leaves with the tool).

Columns, counted on every cycle started in the fault cases (README, Counting):
  failures            cycles whose part did not seat (D + E)
  caught              failures the checks flagged (D)
  silent              failures nobody flagged (E)
  silent per 10,000   E / cycles x 10,000
  good cycles         cycles whose part seated, or would have seated with no checks (A + B + C)
  good stopped        good cycles the checks flagged anyway (B + C)
A transient case counts both the cycles its fault hit and the cycles it did not; a persistent case
counts every cycle of every episode. All of it is a simulation of a stated model.
"""
from __future__ import annotations

import argparse
import json
import sys

# fault class -> family; every class in the campaign must be listed (main() refuses an unknown one)
FAMILIES = [
    ("commands and messages", ("dropped_packet", "duplicated_packet", "reordered_packets", "corrupted_packet",
                               "unauthorized_command", "replayed_command", "malformed_command")),
    ("policy (model) errors", ("model_error_",)),
    ("stale or early camera looks", ("stale_observation_", "look_before_part_arrived")),
    ("processes, calibration, log", ("lost_heartbeat_", "wrong_calibration_id", "frozen_frame", "runtime_hang",
                                     "supervisor_dead", "log_storage_full")),
    ("bad software updates", ("bad_update_",)),
    ("seat and gripper sensors", ("seat_sensor_", "seat_gate_", "gripper_sensor_")),
    ("motors and the nest", ("motor_stall", "torque_over_limit", "part_across_nest")),
    ("mechanics and camera drift", ("base_shift_", "joint_zero_drift_", "lash_growth_", "camera_noise_")),
]


def family(name):
    base = name.split(" (")[0]
    for fam, prefixes in FAMILIES:
        if any(base == p or (p.endswith("_") and base.startswith(p)) for p in prefixes):
            return fam
    raise SystemExit(f"scorecard: fault class {name!r} has no family; add it to FAMILIES")


def cases(d):
    """(class name, counts) for every fault case, transient hit + untouched cycles summed."""
    for k, v in d["transient"].items():
        c = {x: v["on_hit_cycles"][x] + v["on_other_cycles"][x] for x in "ABCDEn"}
        yield k, c
    for k, v in d["persistent"].items():
        yield k, {x: v["counts"][x] for x in "ABCDEn"}


def clean_good(d):
    """Good cycles that no fault acted on, with a seat sensor that never reads a seated part as empty."""
    rows = [v["on_other_cycles"] for k, v in d["transient"].items() if "(" not in k] + \
        [v for k, v in d["fault_free"].items() if k.endswith("p_false 0.0")]
    good = sum(r["A"] + r["B"] + r["C"] for r in rows)
    return good, sum(r["B"] + r["C"] for r in rows)


def row(label, c, classes=""):
    fail, good = c["D"] + c["E"], c["A"] + c["B"] + c["C"]
    per = c["E"] / c["n"] * 1e4 if c["n"] else 0.0
    return (f"| {label}{classes} | {c['n']:,} | {fail:,} | {c['D']:,} | {c['E']:,} | {per:.1f} | {good:,} | "
            f"{c['B'] + c['C']:,} |")


def render(d, by_class=False, out=sys.stdout):
    m = d["meta"]
    units = m["units"]
    cell = m.get("cell", {})
    p = lambda s="": print(s, file=out)
    what = f"Device {cell['device']}" if cell.get("device") else f"Simulated cell, {cell.get('arm', 'model')} arm"
    p(f"{what}, {cell.get('c_mm', 3.0)} mm lead-in. Unit seeds "
      f"{units[0]}-{units[-1]} ({len(units)} units){', QUICK run' if m.get('quick') else ''}. "
      f"Python {m['python']}. Campaign wall time {m.get('wall_s', '?')} s.")
    rows = list(cases(d))
    tot = {x: sum(c[x] for _, c in rows) for x in "ABCDEn"}
    n_classes = len({k.split(' (')[0] for k, _ in rows})
    p(f"{tot['n']:,} cycles in {n_classes} fault classes ({len(rows)} cases: a few classes are rerun with one check "
      f"switched off or with a clock rule, shown in brackets with --by-class).")
    p()
    p("| faults | cycles | failures | caught | silent | silent per 10,000 | good cycles | good stopped |")
    p("|---|---|---|---|---|---|---|---|")
    if by_class:
        for k, c in rows:
            p(row(k, c))
    else:
        for fam, _ in FAMILIES:
            sel = [c for k, c in rows if family(k) == fam]
            names = {k.split(" (")[0] for k, c in rows if family(k) == fam}
            if sel:
                p(row(fam, {x: sum(c[x] for c in sel) for x in "ABCDEn"}, f" ({len(names)})"))
    p(row("**all fault cases**", tot))
    g, stopped = clean_good(d)
    p()
    p(f"Good cycles no fault acted on (untouched cycles of the transient cases, no variants, plus the fault-free "
      f"runs with a truthful seat sensor): {g:,}, of which stopped: {stopped:,}.")
    if m.get("not_run"):
        p()
        p(f"Not run on this device ({len(m['not_run'])}): {', '.join(m['not_run'])}.")
    p()
    render_sensors(d, out)


# the seat-sensor and cling rows: (label, section, key); all from the same campaign run
SENSOR_ROWS = [
    ("fault-free place stack (about 12% of placements fail on the model arm); truthful seat sensor",
     "fault_free", "calibrated lash 0.5 seat p_miss 0.0 p_false 0.0"),
    ("same stack; sensor reads \"seated\" on 1% of unseated parts, \"empty\" on 0.1% of seated ones",
     "fault_free", "calibrated lash 0.5 seat p_miss 0.01 p_false 0.001"),
    ("same stack; the same at 5% and 0.5%",
     "fault_free", "calibrated lash 0.5 seat p_miss 0.05 p_false 0.005"),
    ("tighter stack (about 1 in 10,000 fail on the model arm); 1% of seated parts leave with the tool; last gate read with the tool down",
     "third_read", "cling 0.01 | last read with the tool down"),
    ("the same; last gate read with the tool away",
     "third_read", "cling 0.01 | last read with the tool away"),
]


def render_sensors(d, out=sys.stdout):
    """What the seat sensor's own errors, and a part that leaves with the tool, do to the same counts."""
    p = lambda s="": print(s, file=out)
    rows = [(label, d[sec][key]) for label, sec, key in SENSOR_ROWS if key in d.get(sec, {})]
    if not rows:
        return
    p("| seat sensor and cling cases (no other fault) | cycles | failures | caught | silent | silent per 10,000 "
      "| good cycles | good stopped |")
    p("|---|---|---|---|---|---|---|---|")
    for label, c in rows:
        good, stopped = c["A"] + c["B"] + c["C"], c["B"] + c["C"]
        p(row(label, c)[:-1] + f"({stopped / good:.3%}) |" if good else row(label, c))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="results/campaign.json")
    ap.add_argument("--by-class", action="store_true")
    a = ap.parse_args(argv)
    render(json.load(open(a.path)), a.by_class)


if __name__ == "__main__":
    main()
