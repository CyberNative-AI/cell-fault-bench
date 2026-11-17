"""Fault-injection campaign for the runtime slice. Writes results/campaign.json and results/campaign.md.

    OMP_NUM_THREADS=1 python3 -m cellrt.campaign --out results [--quick] [--procs 6]

Counting on every cycle started:
  A good            seated, not flagged
  B false alarm     seated, flagged
  C false alarm     not seated, flagged, and the same bytes run blind would have seated the part
  D caught failure  not seated, flagged, blind would have failed too
  E silent failure  not seated, not flagged
detect_share = D / (D + E); false_alarm_share = (B + C) / (A + B + C); silent_per_10k = E / N x 10,000.
`prevented` = cycles in A that would have failed blind (a refused forged command, a dropped duplicate).

Simulation of a stated model (cellsim physics, simulated devices). Not a physical measurement.
"""
from __future__ import annotations

import argparse
import json
import os
import statistics as st
import sys
import time
from multiprocessing import Pool

from . import faults as F
from .line import CELL, RUNG_MS, Line, set_cell, supports
from .supervisor import Supervisor

UNITS = list(range(2000, 2020))


def tally(recs, name=None):
    t = dict(n=0, A=0, B=0, C=0, D=0, E=0, prevented=0)
    for r in recs:
        if name is not None and name not in r["faults"]:
            continue
        t["n"] += 1
        if r["seated"]:
            k = "B" if r["flag"] else "A"
            t["prevented"] += (k == "A" and not r["good_blind"])
        elif r["flag"]:
            k = "C" if r["good_blind"] else "D"
        else:
            k = "E"
        t[k] += 1
    return t


def add(a, b):
    return {k: a.get(k, 0) + b.get(k, 0) for k in set(a) | set(b)}


def shares(t):
    f, g = t["D"] + t["E"], t["A"] + t["B"] + t["C"]
    return dict(t, failed=f, detect_share=(round(t["D"] / f, 4) if f >= 30 else None),
                detect_count=f"{t['D']} of {f}", false_alarm_share=round((t["B"] + t["C"]) / g, 5) if g else None,
                silent_per_10k=round(t["E"] / t["n"] * 1e4, 2) if t["n"] else None)


# ---------------------------------------------------------------- 1. fault-free and seat-sensor quality
def _clean(args):
    unit, stack, lash, p_miss, p_false, n = args
    L = Line(unit, stack=stack, lash_deg=lash, ladder=False)
    L.cell.faults.seat_p_miss, L.cell.faults.seat_p_false = p_miss, p_false
    for _ in range(n):
        L.run_cycle()
        L.rt.clear_hold()
    return (stack, lash, p_miss, p_false), tally(L.records), L.cpu_ns, L.log.nbytes, L.now


# ---------------------------------------------------------------- 2. transient faults
TRANSIENT = [
    ("dropped_packet", F.DropPacket, {}), ("duplicated_packet", F.DupPacket, {}),
    ("reordered_packets", F.Reorder, {}), ("corrupted_packet", F.CorruptPacket, {}),
    ("unauthorized_command", F.InjectUnauthorized, {}), ("replayed_command", F.InjectReplay, {}),
    ("malformed_command", F.Malformed, dict(variant="mix")),
    ("model_error_gross", F.ModelGross, {}), ("model_error_units", F.UnitMixup, {}),
    ("model_error_offset_1mm", F.ModelOffset, dict(mm=1.0)), ("model_error_offset_2mm", F.ModelOffset, dict(mm=2.0)),
    ("model_error_offset_3mm", F.ModelOffset, dict(mm=3.0)), ("model_error_offset_4mm", F.ModelOffset, dict(mm=4.0)),
    ("model_error_offset_8mm", F.ModelOffset, dict(mm=8.0)),
    ("stale_observation_static_scene", F.StaleFrame, {}),
    ("look_before_part_arrived", F.EarlyLook, {}),
]


def _transient(args):
    label, unit, n, p, nest_tol = args
    cls, kw = {t[0]: (t[1], t[2]) for t in TRANSIENT}[label]
    L = Line(unit, seed=1, freshness="clock" if nest_tol == "clock" else "event")
    if nest_tol == "clock":
        nest_tol = None
    if nest_tol is not None:
        import dataclasses
        L.rt.lim = dataclasses.replace(L.rt.lim, nest_tol_mm=nest_tol)
    L.inject(F.RecordRelease())
    f = L.inject(cls(p=p, **kw))
    f.name = label
    for _ in range(n):
        L.run_cycle()
    return label, args[4], tally(L.records, label), tally([r for r in L.records if label not in r["faults"]]), dict(L.rt.counts)


# ---------------------------------------------------------------- 3. persistent faults: episodes with the ladder
PERSISTENT = [
    ("lost_heartbeat_policy_dead", F.PolicyDead, {}), ("wrong_calibration_id", F.StaleCalibration, {}),
    ("frozen_frame", F.FrozenCamera, {}), ("runtime_hang", F.RuntimeHung, {}),
    ("supervisor_dead", F.SupervisorDead, {}), ("log_storage_full", F.LogFull, {}),
    ("bad_update_4mm", F.BadUpdate, dict(mm=4.0)), ("bad_update_2mm", F.BadUpdate, dict(mm=2.0)),
    ("bad_update_0.8mm", F.BadUpdate, dict(mm=0.8)),
    ("seat_sensor_high", F.SeatStuck, dict(how="high")), ("seat_sensor_low", F.SeatStuck, dict(how="low")),
    ("seat_sensor_missing", F.SeatStuck, dict(how="missing")),
    ("seat_gate_high", F.SeatGate, dict(how="high")), ("seat_gate_low", F.SeatGate, dict(how="low")),
    ("seat_gate_missing", F.SeatGate, dict(how="missing")), ("gripper_sensor_missing", F.GripperMissing, {}),
    ("motor_stall", F.Stall, {}), ("torque_over_limit", F.TorqueSpike, {}), ("part_across_nest", F.BlockedNest, {}),
    ("base_shift_0.5mm", F.Physical, dict(fault="base_shift_mm", value=0.5)),
    ("base_shift_1mm", F.Physical, dict(fault="base_shift_mm", value=1.0)),
    ("base_shift_2mm", F.Physical, dict(fault="base_shift_mm", value=2.0)),
    ("base_shift_3mm", F.Physical, dict(fault="base_shift_mm", value=3.0)),
    ("joint_zero_drift_0.3deg", F.Physical, dict(fault="offset_drift_deg", value=0.3)),
    ("joint_zero_drift_1deg", F.Physical, dict(fault="offset_drift_deg", value=1.0)),
    ("lash_growth_to_1deg", F.Physical, dict(fault="lash_deg", value=1.0)),
    ("lash_growth_to_2deg", F.Physical, dict(fault="lash_deg", value=2.0)),
    ("camera_noise_1mm", F.Physical, dict(fault="cam_sd_mm", value=1.0)),
    ("camera_noise_2mm", F.Physical, dict(fault="cam_sd_mm", value=2.0)),
]
HANDS = ("clear_nest", "swap_seat_sensor", "swap_gripper", "swap_joint_module", "clean_camera", "free_arm", "reseat_camera")


def _episode(args):
    label, unit, seed, warm, run, variant = args
    cls, kw = {t[0]: (t[1], t[2]) for t in PERSISTENT}[label]
    L = Line(unit, seed=seed, stack=variant.get("stack", "calibrated_uni"))
    if cls is F.SeatGate:
        kw = dict(kw, corner=(unit + seed) % 4)
    if variant.get("no_precheck"):
        L.sup = Supervisor(L.cell.tap(), L.log, hold_line=L.rt.set_hold, heartbeat_line=L.rt.supervisor_heartbeat,
                           precheck=False)
    if variant.get("no_trend"):
        L.sup.trend.h = L.sup.trend.centre_mm = 1e9
    for _ in range(warm):
        L.run_cycle()
    n0, t0, rec0 = len(L.records), L.now, len(L.recoveries)
    f = L.inject(cls(**kw))
    for _ in range(run):
        L.run_cycle()
    ep, evs = L.records[n0:], L.recoveries[rec0:]
    last = ep[-60:]
    hands = [e for e in evs if e["action"] in HANDS]
    return label, dict(
        tally=tally(ep),
        first_flag_s=None if f.first_flag_ms is None else (f.first_flag_ms - t0) / 1e3,
        fault_removed_s=None if f.cleared_ms is None else (f.cleared_ms - t0) / 1e3,
        silent_before_first_flag=f.silent_before_flag,
        cycles_to_first_flag=next((i for i, r in enumerate(ep) if r["t1"] >= (f.first_flag_ms or 1e18)), None),
        recovery_machine_s=sum(e["ms"] for e in evs) / 1e3,
        buyer_hands_actions=len(hands), wrong_hands_actions=sum(1 for e in hands if e["action"] != f.hands),
        hands_named_by_supervisor=sum(1 for e in hands if e.get("named_by") == "supervisor"),

        assists=sum(1 for e in evs if e["rung"] == "assist"),
        recalibrations=sum(1 for e in evs if e["rung"] == "recalibrate"),
        hands_s=sum(RUNG_MS["hands"] for e in hands) / 1e3,
        rungs=[e["action"] for e in evs][:12],
        fail_share_last_60=sum(not r["seated"] for r in last) / len(last),
        flag_share_last_60=sum(r["flag"] for r in last) / len(last))


def _three_gates(args):
    """Cell set to 'flag': one gate goes wrong at cycle 20 and the cell runs on. No ladder, so nothing
    swaps the gate. Counts what the three remaining gates miss, by how the bad seat lay."""
    unit, how, tilt_share, n = args
    L = Line(unit, stack="calibrated", ladder=False, on_gate_fault="flag", seed=5, tilt_share=tilt_share)
    out = dict(cycles=0, good=0, good_flagged=0, failed=0, caught=0, silent=0, silent_tilt_at_masked_corner=0,
               failed_tilt=0, failed_edge=0, failed_no_part=0, notices=0, cycles_before_mask=0, flagged_before_mask=0)
    corner = unit % 4
    for i in range(n):
        if i == 20:
            L.inject(F.SeatGate(corner=corner, how=how))
        r = L.run_cycle()
        L.rt.clear_hold()
        out["notices"] += len(r["notices"])
        if i < 20:
            continue
        if not r["masked"]:
            out["cycles_before_mask"] += 1
            out["flagged_before_mask"] += r["flag"]
            continue
        out["cycles"] += 1
        if r["seated"]:
            out["good"] += 1
            out["good_flagged"] += r["flag"]
            continue
        out["failed"] += 1
        lifted = L.cell.lay              # how it lay under the tool; the interlock has since lifted a caught one away
        out["failed_no_part" if lifted is None else "failed_tilt" if len(lifted) == 1 else "failed_edge"] += 1
        if r["flag"]:
            out["caught"] += 1
        else:
            out["silent"] += 1
            out["silent_tilt_at_masked_corner"] += (L.cell.lifted == (corner,))
    return (how, tilt_share), out


def _cling(args):
    """A seated part leaves with the tool, or rises behind it, on a share `q` of good placements. With and without the read taken once the tool is away."""
    unit, q, away, n = args
    L = Line(unit, ladder=False, away_read=away, seed=6)
    L.cell.faults.cling_p = q
    for _ in range(n):
        L.run_cycle()
        L.rt.clear_hold()
    return (q, away), tally(L.records)


def _interlock(args):
    """A stack that fails about one cycle in eight; a share `block_p` of failed insertions that are
    released lie across the nest. With the release interlock the part is lifted away instead."""
    unit, block_p, interlock, n = args
    L = Line(unit, stack="calibrated", interlock=interlock, seed=7)
    L.cell.faults.block_p = block_p
    nest = 0
    for _ in range(n):
        r = L.run_cycle()
        nest += "seat_precheck" in r["reasons"]
    t = tally(L.records)
    return (block_p, interlock), dict(t, nest_found_blocked=nest, hands=sum(1 for e in L.recoveries if e["hands"]),
                                      hands_s=sum(RUNG_MS["hands"] for e in L.recoveries if e["hands"]) / 1e3,
                                      wall_s=L.now / 1e3)


def med(xs):
    xs = [x for x in xs if x is not None]
    return round(st.median(xs), 2) if xs else None


def summarise_episodes(rows):
    t = {}
    for r in rows:
        t = add(t, r["tally"])
    rung_sets = {}
    for r in rows:
        k = " > ".join(r["rungs"]) or "(none)"
        rung_sets[k] = rung_sets.get(k, 0) + 1
    top = sorted(rung_sets.items(), key=lambda kv: -kv[1])[:3]
    n = len(rows)
    return dict(
        episodes=n, noticed=sum(r["first_flag_s"] is not None for r in rows),
        first_flag_s_median=med([r["first_flag_s"] for r in rows]),
        first_flag_s_max=max([r["first_flag_s"] for r in rows if r["first_flag_s"] is not None], default=None),
        cycles_to_first_flag_median=med([r["cycles_to_first_flag"] for r in rows]),
        silent_failures_before_first_flag_mean=round(sum(r["silent_before_first_flag"] for r in rows) / n, 2),
        fault_removed=sum(r["fault_removed_s"] is not None for r in rows),
        fault_removed_s_median=med([r["fault_removed_s"] for r in rows]),
        recovery_machine_s_median=med([r["recovery_machine_s"] for r in rows]),
        episodes_needing_hands=sum(r["buyer_hands_actions"] > 0 for r in rows),
        hands_actions_mean=round(sum(r["buyer_hands_actions"] for r in rows) / n, 2),
        wrong_hands_actions_total=sum(r["wrong_hands_actions"] for r in rows),
        hands_actions_total=sum(r["buyer_hands_actions"] for r in rows),
        hands_named_by_supervisor_total=sum(r["hands_named_by_supervisor"] for r in rows),

        assists_mean=round(sum(r["assists"] for r in rows) / n, 2),
        recalibrations_total=sum(r["recalibrations"] for r in rows),
        episodes_with_repeat_recalibration=sum(r["recalibrations"] > 1 for r in rows),
        fail_share_last_60=round(sum(r["fail_share_last_60"] for r in rows) / n, 4),
        flag_share_last_60=round(sum(r["flag_share_last_60"] for r in rows) / n, 4),
        counts=shares(t), most_common_ladders=top)


# ---------------------------------------------------------------- 4. combined figure on shared cycles
def _r3(args):
    unit, stack, lash, fault, p_miss, p_false, n = args
    import numpy as np
    L = Line(unit, stack=stack, lash_deg=lash, ladder=False, seed=3)
    L.sup.trend.h = L.sup.trend.centre_mm = 1e9
    L.cell.faults.seat_p_miss, L.cell.faults.seat_p_false = p_miss, p_false
    if fault:
        L.phys.apply_fault(*fault)
    out = dict(n=0, failed=0, good=0, pol_f=0, sup_f=0, both_f=0, pol_g=0, sup_g=0, both_g=0)
    for _ in range(n):
        r = L.run_cycle()
        L.rt.clear_hold()
        xy = L.cell.tap().verify_obs_mm()
        pol = (xy is not None and float(np.hypot(*xy)) > L.cell.c) or ("grasp_empty" in r["reasons"])   # the policy's own verify-look rule, t = 1.0
        sup = r["flag"]
        # Good = seated, or would have seated with no checks. With the release interlock a false "empty"
        # from the sensor sends a good part to the reject place: that is a good cycle stopped, not a
        # failure caught (counting it as caught would show 0.00% false alarms).
        good = r["seated"] or r["good_blind"]
        k = "g" if good else "f"
        out["n"] += 1
        out["good" if good else "failed"] += 1
        out["pol_" + k] += pol
        out["sup_" + k] += sup
        out["both_" + k] += (pol or sup)
    return (stack, lash, fault, p_miss, p_false), out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results")
    ap.add_argument("--quick", action="store_true", help="4 units and short runs, for the test suite")
    ap.add_argument("--procs", type=int, default=6)
    ap.add_argument("--arm", default="model", choices=("model", "p50", "p90"),
                    help="arm: the cell's own (model), or a simulated profile at the per-joint p50 or p90 of public SO-100/101 recordings (cellsim/arms.py)")
    ap.add_argument("--c-mm", type=float, default=3.0, help="fixture lead-in, mm")
    ap.add_argument("--device", default=None, metavar="MODULE:FACTORY",
                    help="run the bench against your own cell (README, \"Run the bench against your cell\"); "
                         "runs the fault classes the device SUPPORTS and lists the rest as not run")
    a = ap.parse_args(argv)
    set_cell(a.c_mm, a.arm, a.device)
    can = supports(a.device)
    runs = lambda cls: getattr(cls, "needs", None) in (None, *can)
    os.makedirs(a.out, exist_ok=True)
    units = UNITS[:4] if a.quick else UNITS
    n_cyc = 100 if a.quick else 500
    t_start = time.time()
    doc = dict(meta=dict(written_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), units=units, quick=a.quick,
                         python=sys.version.split()[0], kind="SIM: cellsim physics + simulated devices",
                         definition="counted on every cycle started (README, Counting)", cell=dict(CELL)))
    with Pool(a.procs, initializer=set_cell, initargs=(a.c_mm, a.arm, a.device)) as pool:
        # 1
        grid = [("calibrated_uni", 0.5, 0.0, 0.0), ("calibrated_uni", 1.0, 0.0, 0.0), ("calibrated", 0.5, 0.0, 0.0),
                ("calibrated", 0.5, 0.01, 0.001), ("calibrated", 0.5, 0.05, 0.005), ("calibrated_uni", 0.5, 0.01, 0.001)]
        if a.device:                               # stack and lash are the example cell's settings
            grid = [("device", 0, 0.0, 0.0)] + ([("device", 0, 0.01, 0.001), ("device", 0, 0.05, 0.005)]
                                                 if {"seat_p_miss", "seat_p_false"} <= can else [])
        res = pool.map(_clean, [(u, *g, n_cyc) for g in grid for u in units])
        clean = {}
        for g in grid:
            rows = [r for r in res if r[0] == g]
            t = {}
            for r in rows:
                t = add(t, r[1])
            cyc = t["n"]
            clean[(f"{g[0]} lash {g[1]}" if g[0] != "device" else "device") + f" seat p_miss {g[2]} p_false {g[3]}"] = dict(
                shares(t), cpu_us_per_cycle=round(sum(r[2] for r in rows) / cyc / 1e3, 1),
                log_bytes_per_cycle=round(sum(r[3] for r in rows) / cyc), cycle_s=round(sum(r[4] for r in rows) / cyc / 1e3, 3))
        doc["fault_free"] = clean
        # 2
        jobs = [(t[0], u, n_cyc, 0.05, None) for t in TRANSIENT if runs(t[1]) for u in units]
        jobs += [(lab, u, n_cyc, 0.05, "clock") for lab in ("stale_observation_static_scene", "look_before_part_arrived")
                 for u in units]
        jobs += [(lab, u, n_cyc, 0.05, 1e9) for lab in ("model_error_offset_2mm", "model_error_offset_3mm",
                                                         "model_error_offset_4mm", "model_error_offset_8mm") for u in units]
        res = pool.map(_transient, jobs)
        tr = {}
        for lab, tol in sorted({(r[0], r[1]) for r in res}, key=lambda x: ({None: 0, "clock": 1}.get(x[1], 2), [t[0] for t in TRANSIENT].index(x[0]))):
            rows = [r for r in res if (r[0], r[1]) == (lab, tol)]
            hit, non, counts = {}, {}, {}
            for r in rows:
                hit, non, counts = add(hit, r[2]), add(non, r[3]), add(counts, r[4])
            tr[lab + ("" if tol is None else " (clock freshness)" if tol == "clock" else " (cell-map check off)")] = dict(on_hit_cycles=shares(hit),
                                                                               on_other_cycles=shares(non), refusals=counts)
        doc["transient"] = tr
        # 3
        seeds = (0,) if a.quick else (0, 1)
        jobs = [(t[0], u, s, 80, 150, {}) for t in PERSISTENT if runs(t[1]) for u in units for s in seeds]
        res = pool.map(_episode, jobs)
        doc["persistent"] = {t[0]: summarise_episodes([r[1] for r in res if r[0] == t[0]]) for t in PERSISTENT if runs(t[1])}
        doc["meta"]["not_run"] = [t[0] for t in TRANSIENT + PERSISTENT if not runs(t[1])]
        if a.device:                               # the sections below vary the example cell's own settings
            doc["meta"]["not_run"] += ["ablations", "three_gates", "third_read", "interlock", "combined_r3"]
        if not a.device:
            # ablations: what each check buys
            abl = {}
            for lab, variant, name in (("seat_sensor_high", dict(stack="calibrated"), "seat stuck high, stack that fails 13%: with the pre-check"),
                                       ("seat_sensor_high", dict(stack="calibrated", no_precheck=True), "same, pre-check removed"),
                                       ("base_shift_1mm", dict(no_trend=True), "base shift 1 mm, trend monitor removed"),
                                       ("bad_update_2mm", dict(no_trend=True), "bad update 2 mm, trend monitor removed"),
                                       ("lash_growth_to_2deg", dict(no_trend=True), "lash growth to 2 deg, trend monitor removed")):
                rr = pool.map(_episode, [(lab, u, s, 80, 150, variant) for u in units for s in seeds])
                abl[name] = summarise_episodes([r[1] for r in rr])
            doc["ablations"] = abl
            # one bad gate of four, masked, on a stack that fails about one cycle in eight: what running on three costs
            cases = [(how, ts) for ts in (None, 0.333) for how in ("high", "low", "missing")]
            res = pool.map(_three_gates, [(u, how, ts, n_cyc) for how, ts in cases for u in units])
            gates = {}
            for how, ts in cases:
                t = {}
                for r in res:
                    if r[0] == (how, ts):
                        t = add(t, r[1])
                lie = "bad seats lie as the cell model gives" if ts is None else "a third of bad seats lift one corner only (stated fixture mix)"
                gates[f"one gate {how}, masked; {lie}"] = t
            doc["three_gates"] = gates
            # the third read (tool away) and the release interlock (tool down)
            cases = [(q, away) for q in (0.0, 0.001, 0.01, 0.05) for away in (False, True)]
            res = pool.map(_cling, [(u, q, away, n_cyc) for q, away in cases for u in units])
            cl = {}
            for c in cases:
                t = {}
                for r in res:
                    if r[0] == c:
                        t = add(t, r[1])
                cl[f"cling {c[0]} | last read with the tool {'away' if c[1] else 'down'}"] = shares(t)
            doc["third_read"] = cl
            cases = [(bp, il) for bp in (0.1, 0.3) for il in (False, True)]
            res = pool.map(_interlock, [(u, bp, il, n_cyc) for bp, il in cases for u in units])
            il = {}
            for c in cases:
                t = {}
                for r in res:
                    if r[0] == c:
                        t = add(t, r[1])
                il[f"left across the nest {c[0]} | interlock {'on' if c[1] else 'off'}"] = dict(shares(t), nest_found_blocked=t["nest_found_blocked"],
                                                                                             hands=t["hands"], hands_s=t["hands_s"], wall_s=round(t["wall_s"]))
            doc["interlock"] = il
            # 4
            cells = []
            for stack, lash, fault in (("calibrated", 0.5, None), ("calibrated_uni", 1.0, None),
                                       ("calibrated_uni", 0.5, ("base_shift_mm", 1.5)), ("calibrated_uni", 0.5, ("base_shift_mm", 2.0)),
                                       ("servo_cal", 1.0, None)):
                for pm, pf in ((0.0, 0.0), (0.01, 0.001), (0.05, 0.005)):
                    cells.append((stack, lash, fault, pm, pf))
            res = pool.map(_r3, [(u, *c, n_cyc) for c in cells for u in units])
            r3 = {}
            for c in cells:
                t = {}
                for r in res:
                    if r[0] == c:
                        t = add(t, r[1])
                f, g = max(1, t["failed"]), max(1, t["good"])
                name = f"{c[0]} lash {c[1]}" + (f" + {c[2][0]}={c[2][1]}" if c[2] else "") + f" | seat p_miss {c[3]} p_false {c[4]}"
                row = dict(cycles=t["n"], failed=t["failed"])
                for who in ("pol", "sup", "both"):
                    row[{"pol": "policy_self_check", "sup": "supervisor", "both": "combined"}[who]] = dict(
                        detect=(round(t[who + "_f"] / f, 4) if t["failed"] >= 30 else f"{t[who + '_f']} of {t['failed']}"),
                        false_alarm=round(t[who + "_g"] / g, 5),
                        silent_per_10k=round((t["failed"] - t[who + "_f"]) / t["n"] * 1e4, 1))
                r3[name] = row
            doc["combined_r3"] = r3
    doc["meta"]["wall_s"] = round(time.time() - t_start, 1)
    with open(os.path.join(a.out, "campaign.json"), "w") as fh:
        json.dump(doc, fh, indent=1)
    print(f"campaign: {doc['meta']['wall_s']} s -> {os.path.join(a.out, 'campaign.json')}\n")
    from .scorecard import render
    render(doc)


if __name__ == "__main__":
    main()
