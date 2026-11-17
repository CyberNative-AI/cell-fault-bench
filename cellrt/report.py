"""Render results/campaign.json as tables.

    python3 -m cellrt.report results/campaign.json > results/campaign.md
"""
from __future__ import annotations

import json
import sys


def pct(x, d=1):
    return "-" if x is None else f"{100 * x:.{d}f}%"


def det(c):
    return pct(c["detect_share"]) + f" ({c['detect_count']})" if c["detect_share"] is not None else c["detect_count"]


def main(path):
    d = json.load(open(path))
    m = d["meta"]
    print(f"# Fault-injection campaign\n")
    print(f"Written {m['written_utc']}; {len(m['units'])} simulated units ({m['units'][0]}-{m['units'][-1]}); wall {m['wall_s']} s; "
          f"Python {m['python']}. **{m['kind']}.** Not a physical measurement; software checks are not engineered limits.\n")
    print("Counting on every cycle started. A = seated, not "
          "flagged. B = seated, flagged. C = not seated, flagged, and the same bytes run with no checks would have "
          "seated the part. D = not seated, flagged, would have failed anyway. E = not seated, not flagged (silent). "
          "detect = D/(D+E); false alarm = (B+C)/(A+B+C); a share over fewer than 30 failures is given as a count. "
          "\"Prevented\" = cycles that ended good and would have failed with no checks.\n")
    print("**Base-shift sizes are not physical sizes.** In this cell the part's offset in the gripper adds to the place error where "
          "physically it subtracts (`cellrt/plant.py`, place branch of `SimCell.move`). A moved base shows in the verify reading at 0.48 times the bump, not 1.45 times, so every "
          "`base_shift` row here, the trend-monitor rows and the calibration counts stand for a bump about three times "
          "larger than its label (the 1 mm row is about 3 mm). Which check catches what is not affected; how small a bump "
          "matters is. Kept because every number in this bench was tuned and run on it.\n")

    print("## 1. No injected fault: what the supervisor costs and what the seat sensor's quality does\n")
    print("| stack, lash, seat sensor error (miss / false) | cycles | failed | detect | false alarm | silent per 10,000 | CPU per cycle | log per cycle |")
    print("|---|---|---|---|---|---|---|---|")
    for k, c in d["fault_free"].items():
        print(f"| {k} | {c['n']} | {c['failed']} | {det(c)} | {pct(c['false_alarm_share'], 3)} | {c['silent_per_10k']} | "
              f"{c['cpu_us_per_cycle']} µs | {c['log_bytes_per_cycle']} B |")
    c0 = next(iter(d["fault_free"].values()))
    print(f"\nCycle time {c0['cycle_s']} s (the cell model's timing for the one-direction stack). The checks add no motion and no camera "
          f"look; the runtime, supervisor and log use about {c0['cpu_us_per_cycle']} µs of one desktop core per cycle "
          f"(Python; a small computer will be 10-50 times slower and it still does not matter) and "
          f"{c0['log_bytes_per_cycle']} bytes of log.\n")

    print("## 2. Transient faults, each on about 5% of cycles\n")
    print("| fault | cycles hit | A good | B+C false alarm | D caught | E silent | prevented | detect | false alarms on the other cycles |")
    print("|---|---|---|---|---|---|---|---|---|")
    for k, v in d["transient"].items():
        h, o = v["on_hit_cycles"], v["on_other_cycles"]
        print(f"| {k} | {h['n']} | {h['A']} | {h['B'] + h['C']} | {h['D']} | {h['E']} | {h['prevented']} | {det(h)} | "
              f"{o['B'] + o['C']} of {o['A'] + o['B'] + o['C']} |")

    print("\n## 3. Persistent faults: episodes with the recovery ladder\n")
    print("80 fault-free cycles, then the fault, then 150 cycles. Times are simulated seconds from the fault's onset. "
          "\"Recovery time\" is machine time spent on ladder rungs (rung times: `cellrt/line.py` RUNG_MS, each tagged CALC "
          "or ESTIMATE); hands = a person had to do something at the machine.\n")
    print("| fault | episodes | noticed | first flag, median s (max) | cycles to first flag | silent failures before it | "
          "fault removed | recovery time, median s | needed hands | wrong first hands action | failing in the last 60 cycles | usual ladder |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for k, v in d["persistent"].items():
        mx = "-" if v["first_flag_s_max"] is None else f"{v['first_flag_s_max']:.0f}"
        print(f"| {k} | {v['episodes']} | {v['noticed']} | {v['first_flag_s_median']} ({mx}) | {v['cycles_to_first_flag_median']} | "
              f"{v['silent_failures_before_first_flag_mean']} | {v['fault_removed']} | {v['recovery_machine_s_median']} | "
              f"{v['episodes_needing_hands']} | {v['wrong_hands_actions_total']} | {pct(v['fail_share_last_60'], 2)} | "
              f"{v['most_common_ladders'][0][0]} ({v['most_common_ladders'][0][1]}) |")
    print("\nCounts over all cycles of these episodes:\n")
    print("| fault | cycles | A | B | C | D | E silent | detect | false alarm | silent per 10,000 |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for k, v in d["persistent"].items():
        c = v["counts"]
        print(f"| {k} | {c['n']} | {c['A']} | {c['B']} | {c['C']} | {c['D']} | {c['E']} | {det(c)} | {pct(c['false_alarm_share'], 2)} | {c['silent_per_10k']} |")

    rec = sum(v.get("recalibrations_total", 0) for v in d["persistent"].values())
    rep = sum(v.get("episodes_with_repeat_recalibration", 0) for v in d["persistent"].values())
    print(f"\nRecalibrations across all episodes: {rec}; episodes in which the first one was not enough and the ladder "
          f"recalibrated again: {rep}. The automatic calibration is noisy; the centre check is what notices.\n")
    print("\n## 4. What each check buys (ablations)\n")
    print("| case | episodes | noticed | first flag, median s | silent failures before first flag | cycles | caught | silent | silent per 10,000 | failing at the end |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for k, v in d["ablations"].items():
        c = v["counts"]
        print(f"| {k} | {v['episodes']} | {v['noticed']} | {v['first_flag_s_median']} | {v['silent_failures_before_first_flag_mean']} | "
              f"{c['n']} | {c['D']} | {c['E']} | {c['silent_per_10k']} | {pct(v['fail_share_last_60'], 2)} |")

    print("\n## 5. One combined detection figure on shared cycles\n")
    print("Policy self-check = the policy's own rule (one verify look, flag when the offset exceeds the slack, plus an empty "
          "gripper). Supervisor = the seat-sensor transition check. Combined = flagged by either, counted once. "
          "A cycle is good if the part seated or would have seated with no checks: with the release interlock a false "
          "\"empty\" from the sensor sends a good part to the reject place, which is a good cycle stopped.\n")
    print("| cell and seat sensor error | cycles | failed | policy: detect / false alarm / silent per 10k | supervisor | combined |")
    print("|---|---|---|---|---|---|")
    f3 = lambda x: f"{x['detect'] if isinstance(x['detect'], str) else pct(x['detect'])} / {pct(x['false_alarm'], 2)} / {x['silent_per_10k']}"
    for k, v in d["combined_r3"].items():
        print(f"| {k} | {v['cycles']} | {v['failed']} | {f3(v['policy_self_check'])} | {f3(v['supervisor'])} | {f3(v['combined'])} |")

    if "three_gates" in d:
        print("\n## 6. Four seat gates: naming a bad gate, and what running on three costs\n")
        print("The nest carries four corner gates (`seat[0..3]`). Exactly one corner high before a "
              "load is a gate (a part left in the nest presses at least two); the same single corner low three cycles "
              "running is a gate. The supervisor then names the part to change itself.\n")
        print("| fault (one working stack, with the ladder) | episodes | cycles to first flag | silent failures | "
              "hands actions | named by the supervisor | wrong hands actions |")
        print("|---|---|---|---|---|---|---|")
        for k in ("seat_sensor_high", "seat_sensor_low", "seat_sensor_missing", "seat_gate_high", "seat_gate_low",
                  "seat_gate_missing", "part_across_nest"):
            v = d["persistent"][k]
            print(f"| {k} | {v['episodes']} | {v['cycles_to_first_flag_median']} | {v['counts']['E']} | {v['hands_actions_total']} | "
                  f"{v['hands_named_by_supervisor_total']} | {v['wrong_hands_actions_total']} |")
        print("\nCell set to \"flag\" instead of \"stop\": the bad gate is masked, one notice is raised and the cell runs "
              "on three gates. Stack that fails about one cycle in eight, no ladder, so nothing swaps the gate. How a bad "
              "seat lies: a tilt lifts one corner, a part hung on the lead-in lifts one edge.\n")
        print("| case | notices | cycles on three gates | good, flagged | failed | bad seat lay tilted / on an edge / no part reached the nest | caught | "
              "silent | silent: tilt at the masked corner | silent per 10,000 |")
        print("|---|---|---|---|---|---|---|---|---|---|")
        for k, t in d["three_gates"].items():
            print(f"| {k} | {t['notices']} | {t['cycles']} | {t['good_flagged']} of {t['good']} | {t['failed']} | "
                  f"{t['failed_tilt']} / {t['failed_edge']} / {t['failed_no_part']} | {t['caught']} | {t['silent']} | "
                  f"{t['silent_tilt_at_masked_corner']} | {round(t['silent'] / max(1, t['cycles']) * 1e4, 1)} |")
        print("\nOn three gates the cell misses a bad seat whose only lifted corner is the masked one, and nothing else: "
              "a quarter of single-corner lifts. How often a bad seat is a single-corner lift is not measured. This bench's "
              "plant, fed the cell model's placement errors, gives none at this lash; the stated fixture mix gives a third of attempted bad seats "
              "(CALC for that mix: 1/3 x 1/4, about 8% of failures silent). The three gate-fault modes rerun the same units "
              "and placements: the rows are one sample of about 1,180 failures seen three ways, not three samples. "
              "The rows named for the cell model's physics use this bench's own mapping of a lateral miss to a part hung with one edge "
              "up (`cellrt/plant.py`): the cell model has no state for how a failed part lies, "
              "so it does not say \"none\", it says nothing. The working "
              "assumption for the value of the fourth gate is therefore the stated fixture mix, the less favourable one.")

    if "third_read" in d:
        print("\n## 7. Three seat reads, and a release that waits for the gates\n")
        print("The gates are read before "
              "the load, with the tool down and the vacuum on, and again once the tool is away. `cling` is the share of "
              "seated parts that leave with the tool or rise behind it; nobody has measured it.\n")
        print("| cling, and where the last read is taken | cycles | failed | caught | silent | silent per 10,000 | good cycles flagged |")
        print("|---|---|---|---|---|---|---|")
        for k, c in d["third_read"].items():
            print(f"| {k} | {c['n']} | {c['failed']} | {c['D']} | {c['E']} | {c['silent_per_10k']} | {c['B'] + c['C']} |")
        print("\nThe release interlock: when the gates do not all close with the tool down, the supervisor refuses that "
              "one release, and the part goes back up on the cup to the reject place. Stack that fails about one cycle in "
              "eight, with the ladder; `left across the nest` is the assumed share of failed insertions that, once "
              "released, lie across the nest and need a person.\n")
        print("| case | cycles | failed | silent | cycles that found the nest blocked | hands actions | person-minutes | cell time, hours |")
        print("|---|---|---|---|---|---|---|---|")
        for k, c in d["interlock"].items():
            print(f"| {k} | {c['n']} | {c['failed']} | {c['E']} | {c['nest_found_blocked']} | {c['hands']} | "
                  f"{round(c['hands_s'] / 60)} | {round(c['wall_s'] / 3600, 1)} |")
        print("\nHands actions that remain with the interlock on are the ladder asking for help with a stack that keeps "
              "failing, not a blocked nest. A part that wedges and stays down when the tool lifts with the vacuum on is "
              "not in these rows (`wedge_p`, default 0, unmeasured).")

    tr = d["transient"]
    if "stale_observation_static_scene (clock freshness)" in tr:
        print("\n## 8. Freshness by event or by clock\n")
        print("Event: a look is usable if it was taken after the last change to the nest it looks at (a part arrived, a "
              "part was released or cleared, hands or a recovery in the cell), up to a 300 s backstop. Clock: its age must "
              "not exceed the request's own deadline (10 s here). Each case on about 5% of cycles.\n")
        print("| case | rule | cycles hit | ran and seated | refused though the part would have seated | failed and caught | silent |")
        print("|---|---|---|---|---|---|---|")
        for lab, what in (("stale_observation_static_scene", "look 30 s old, nest unchanged since"),
                          ("look_before_part_arrived", "look 3 s old, taken before the part arrived")):
            for rule, key in (("event", lab), ("clock", lab + " (clock freshness)")):
                h = tr[key]["on_hit_cycles"]
                print(f"| {what} | {rule} | {h['n']} | {h['A']} | {h['B'] + h['C']} | {h['D']} | {h['E']} |")
        print("\nIn a fixtured cell every part lands in the same nest, so this model cannot show harm from a look taken "
              "before the part arrived: every such refusal counts here as a good cycle lost. The rule is for the case the "
              "model does not have, a pick target read from the camera.")

if __name__ == "__main__":
    main(sys.argv[1])
