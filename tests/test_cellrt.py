"""Tests for the runtime slice. Run from the repository root:

    python3 -m unittest discover -s tests

Each test names the property it holds, not only a case.
"""
import ast
import dataclasses
import json
import math
import os
import random
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from cellrt import contracts as C  # noqa: E402
from cellrt import faults as F  # noqa: E402
from cellrt import release as R  # noqa: E402
from cellrt.eventlog import verify  # noqa: E402
from cellrt.line import KEY, KID, Line  # noqa: E402
from cellrt.replay import replay  # noqa: E402
from cellrt.runtime import Limits, admit_static  # noqa: E402

GOOD = dict(cycle=1, step=1, kind="place", target_xyz_mm=[190.0, 60.0, 30.0], approach="uni", capture_radius_mm=3.0,
            expected_error_mm_p99=1.5, calibration_id="cal-1", policy_id="transfer", policy_version="1.0.0",
            deadline_ms=10000, observed_at_ms=0, frame_id=1)


class Contracts(unittest.TestCase):
    def test_good_request_parses(self):
        self.assertEqual(C.parse_request(dict(GOOD)).kind, "place")

    def test_no_input_yields_a_nonfinite_or_mistyped_request(self):
        """Property: parse_request either raises Malformed or returns only finite, typed fields."""
        rnd = random.Random(7)
        junk = [None, True, False, "x", "", [], {}, [1, 2], float("nan"), float("inf"), -1, 10 ** 30, 1e308, "3.0", b"x"]
        accepted = 0
        for _ in range(20000):
            d = dict(GOOD)
            for _ in range(rnd.randint(1, 3)):
                k = rnd.choice(list(d) + ["extra"])
                op = rnd.random()
                if op < 0.15 and k in d:
                    del d[k]
                elif k == "target_xyz_mm" and op < 0.6:
                    d[k] = [rnd.choice(junk + [1.0, 2.0]) for _ in range(rnd.randint(0, 4))]
                else:
                    d[k] = rnd.choice(junk)
            try:
                r = C.parse_request(d)
            except C.Malformed:
                continue
            accepted += 1
            self.assertTrue(all(isinstance(v, float) and math.isfinite(v) for v in r.target_xyz_mm))
            self.assertTrue(math.isfinite(r.capture_radius_mm) and math.isfinite(r.expected_error_mm_p99))
            self.assertIn(r.kind, C.KINDS)
            self.assertEqual(r.step, C.STEP_OF[r.kind])
            for k in ("cycle", "deadline_ms", "observed_at_ms", "frame_id"):
                self.assertIs(type(getattr(r, k)), int)
        self.assertLess(accepted, 20000)

    def test_envelope_rejects_any_changed_byte(self):
        env = C.seal(KEY, KID, 5, GOOD)
        self.assertEqual(C.open_envelope({KID: KEY}, env)[0], 5)
        for field in ("body", "mac", "key_id"):
            for i in range(0, len(env[field]), 7):
                bad = dict(env)
                ch = "A" if env[field][i] != "A" else "B"
                bad[field] = env[field][:i] + ch + env[field][i + 1:]
                with self.assertRaises((PermissionError, C.Malformed)):
                    C.open_envelope({KID: KEY}, bad)
        with self.assertRaises(PermissionError):
            C.open_envelope({KID: KEY}, dict(env, seq=6))

    def test_nan_token_in_a_signed_body_is_malformed_not_executed(self):
        L = Line(2000)
        text = json.dumps(dict(GOOD, cycle=0, step=0, kind="grasp", target_xyz_mm=[float("nan"), -60.0, 30.0]))
        L.sup.heartbeat(0)
        rep = L.rt.submit(L._seal(text), 10)
        self.assertEqual((rep.outcome, rep.reason), ("refused", "malformed"))


class RuntimeChecks(unittest.TestCase):
    def cycle_reqs(self, L):
        return L.policy.requests(L.cycle, L.now, L.frame + 1)

    def test_clean_cycles_seat_and_nothing_is_flagged(self):
        L = Line(2000)
        for _ in range(50):
            L.run_cycle()
        self.assertEqual(sum(r["seated"] for r in L.records), 50)
        self.assertEqual(sum(r["flag"] for r in L.records), 0)

    def test_each_refusal_reason_fires_and_nothing_moves(self):
        reasons = ("unauthorized", "replay", "stale_calibration", "unknown_policy", "stale_observation", "envelope",
                   "not_a_nest", "error_exceeds_capture", "out_of_sequence", "supervisor_hold", "no_supervisor",
                   "stop_input", "frozen_frame", "log_unavailable")
        for reason in reasons:
            L = Line(2000)
            L.sup.heartbeat(0)
            r = self.cycle_reqs(L)[0]
            now = 10
            if reason == "replay":
                e = L._seal(dict(r, target_xyz_mm=[900.0, 0.0, 30.0]))      # refused for another reason first
                L.rt.submit(e, now)
                rep = L.rt.submit(e, now + 1)
                self.assertEqual(rep.reason, reason)
                self.assertEqual(L.cell.at, "home")
                continue
            if reason == "stale_calibration":
                r["calibration_id"] = "cal-0"
            elif reason == "unknown_policy":
                r["policy_version"] = "9.9.9"
            elif reason == "stale_observation":
                now = 6000
                L.rt.scene_event("src", 5000)                              # the source nest changed after the look
                L.sup.heartbeat(now)
            elif reason == "envelope":
                r["target_xyz_mm"] = [900.0, 0.0, 30.0]
            elif reason == "not_a_nest":
                r["target_xyz_mm"] = [190.0, -55.0, 30.0]
            elif reason == "error_exceeds_capture":
                r["expected_error_mm_p99"] = 5.0
            elif reason == "out_of_sequence":
                r = self.cycle_reqs(L)[2]
            elif reason == "supervisor_hold":
                L.rt.set_hold("test")
            elif reason == "no_supervisor":
                now = 5000
            elif reason == "stop_input":
                L.rt.stop_input = True
            elif reason == "frozen_frame":
                L.rt.last_grasp_frame = 99
            elif reason == "log_unavailable":
                L.log.capacity = len(L.log.lines)
            e = L._seal(r)
            if reason == "unauthorized":
                e = dict(e, mac="0" * 64)
            rep = L.rt.submit(e, now)
            self.assertEqual(rep.reason, reason, reason)
            self.assertFalse(L.cell.holding, reason)
            self.assertEqual(L.cell.at, "home", reason)

    def test_unauthenticated_bytes_never_change_state(self):
        """Property: after any number of forged or garbage envelopes, a legitimate cycle still completes."""
        L = Line(2000)
        rnd = random.Random(3)
        L.cell.begin_cycle()
        L.sup.heartbeat(0)
        v = L.sup.begin_cycle(0, 0)
        now = 0
        for r in self.cycle_reqs(L):
            for _ in range(20):
                junk = rnd.choice([None, 7, "x", {}, {"key_id": KID, "seq": 10 ** 6, "body": "{}", "mac": "00"},
                                   {"key_id": "nobody", "seq": 1, "body": json.dumps(GOOD), "mac": "0" * 64},
                                   dict(C.seal(b"wrong-key", KID, 10 ** 6, dict(GOOD, kind="release", step=2)))])
                now += 1
                L.sup.heartbeat(now)
                rep = L.rt.submit(junk, now)
                self.assertIn(rep.reason, ("unauthorized", "malformed"))
            now += 10
            L.sup.heartbeat(now)
            rep = L.rt.submit(L._seal(r), now)
            self.assertEqual(rep.outcome, "done")
            L.sup.on_reply(rep, now)
            now += rep.duration_ms
        self.assertTrue(L.cell.truth()["seated"])
        self.assertFalse(v.flag)

    def test_supervisor_imports_neither_runtime_nor_policy(self):
        with open(os.path.join(HERE, "..", "cellrt", "supervisor.py")) as fh:
            tree = ast.parse(fh.read())
        names = set()
        for n in ast.walk(tree):
            if isinstance(n, ast.ImportFrom):
                names.add(n.module or "")
            elif isinstance(n, ast.Import):
                names |= {a.name for a in n.names}
        self.assertFalse({x for x in names if any(w in x for w in ("runtime", "policy", "line", "plant"))}, names)

    def test_tap_has_no_motion(self):
        self.assertFalse(hasattr(Line(2000).cell.tap(), "do"))


class FaultsAndLadder(unittest.TestCase):
    def episode(self, fault, warm=60, run=80, unit=2003):
        L = Line(unit)
        for _ in range(warm):
            L.run_cycle()
        n0 = len(L.records)
        f = L.inject(fault)
        for _ in range(run):
            L.run_cycle()
        return L, f, L.records[n0:]

    def test_no_silent_failure_under_any_runtime_fault(self):
        """Property: with a truthful seat sensor, no cycle ends unseated and unflagged, whatever the wire,
        the policy or the processes do."""
        made = [F.DropPacket(p=0.3, step=None), F.DupPacket(p=0.3), F.Reorder(p=0.3), F.CorruptPacket(p=0.3),
                F.InjectUnauthorized(p=0.3), F.Malformed(p=0.3, variant="mix"), F.ModelGross(p=0.3),
                F.ModelOffset(p=0.3, mm=2.0), F.ModelOffset(p=0.3, mm=6.0), F.UnitMixup(p=0.3), F.StaleFrame(p=0.3),
                F.PolicyDead(), F.StaleCalibration(), F.FrozenCamera(), F.RuntimeHung(), F.SupervisorDead(), F.LogFull(),
                F.BadUpdate(mm=4.0), F.Stall(), F.TorqueSpike(), F.GripperMissing(), F.BlockedNest(),
                F.SeatStuck(how="low"), F.SeatStuck(how="missing"), F.SeatStuck(how="high"), F.EarlyLook(p=0.3)]
        for fault in made:
            L, f, ep = self.episode(fault, warm=10, run=60)
            self.assertEqual([r for r in ep if not r["seated"] and not r["flag"]], [], f.name)

    def test_forged_and_replayed_commands_are_refused_and_work_goes_on(self):
        for cls in (F.InjectUnauthorized, F.InjectReplay):
            L = Line(2004)
            L.inject(F.RecordRelease())
            f = L.inject(cls(p=0.5))
            for _ in range(60):
                L.run_cycle()
            hit = [r for r in L.records if f.name in r["faults"]]
            self.assertGreater(len(hit), 10)
            self.assertTrue(all(r["seated"] and not r["flag"] and not r["good_blind"] for r in hit), f.name)

    def test_stuck_high_seat_sensor_is_found_before_the_first_place(self):
        L, f, ep = self.episode(F.SeatStuck(how="high"))
        self.assertEqual(ep[0]["reasons"], ["seat_precheck"])
        self.assertNotIn(f, L.faults)

    # ---------------------------------------------------------------- four seat gates (seat[0..3])
    def senses(self, L, at):
        return [json.loads(x)["d"]["seat"] for x in L.log.lines if '"k":"sense"' in x and f'"at":"{at}"' in x]

    def test_cycle_record_carries_four_seat_bits(self):
        L = Line(2000)
        for _ in range(5):
            L.run_cycle()
        self.assertEqual(self.senses(L, "pre"), ["0000"] * 5)           # before the load
        self.assertEqual(self.senses(L, "down"), ["1111"] * 5)          # tool down, vacuum on
        self.assertEqual(self.senses(L, "away"), ["1111"] * 5)          # tool clear of the part

    def test_one_gate_high_before_a_load_is_named_as_a_gate_and_a_left_part_as_a_part(self):
        """Property: exactly one corner high at the pre-load read names the gate; two or more is a part
        in the nest. The supervisor names the gate itself: no wrong hands action, whatever the corner."""
        for corner in range(4):
            L, f, ep = self.episode(F.SeatGate(corner=corner, how="high"), warm=5, run=10)
            self.assertEqual(ep[0]["reasons"], ["seat_gate_stuck"])
            self.assertEqual(L.sup.bad_gate, None)                          # cleared by the swap
            hands = [e for e in L.recoveries if e["hands"]]
            self.assertEqual([(e["action"], e["named_by"]) for e in hands], [("swap_seat_sensor", "supervisor")])
            self.assertEqual(self.senses(L, "pre")[5], "".join("1" if i == corner else "0" for i in range(4)))
            # a gate that gives no reading is named the same way; all four silent is the connector, and goes to the agent
            L, f, ep = self.episode(F.SeatGate(corner=corner, how="missing"), warm=5, run=10)
            self.assertEqual(ep[0]["reasons"], ["seat_gate_missing"])
            self.assertEqual([(e["action"], e["named_by"]) for e in L.recoveries if e["hands"]], [("swap_seat_sensor", "supervisor")])
        L, f, ep = self.episode(F.SeatStuck(how="missing"), warm=5, run=10)
        self.assertEqual(ep[0]["reasons"], ["seat_missing"])
        self.assertTrue(all(e["named_by"] == "agent" for e in L.recoveries if e["hands"]))
        L, f, ep = self.episode(F.BlockedNest(), warm=5, run=10)
        self.assertEqual(ep[0]["reasons"], ["seat_precheck"])
        self.assertEqual(self.senses(L, "pre")[5].count("1"), 2)

    def test_one_gate_low_is_named_on_the_third_cycle_without_a_recalibration(self):
        for corner in range(4):
            L, f, ep = self.episode(F.SeatGate(corner=corner, how="low"), warm=5, run=20)
            self.assertEqual([r["reasons"] for r in ep[:4]],
                             [["not_seated"], ["not_seated"], ["not_seated", "seat_gate_stuck"], []])
            self.assertEqual([e["action"] for e in L.recoveries], ["retry", "retry", "swap_seat_sensor"])
            self.assertNotIn(f, L.faults)

    def test_no_silent_failure_with_one_bad_gate_when_the_cell_stops_on_it(self):
        """Property (stop setting, a stack that fails about one cycle in eight): one gate wrong in any
        corner and any way never lets a cycle end unseated and unflagged."""
        for corner in range(4):
            for how in ("high", "low", "missing"):
                L = Line(2003, stack="calibrated")
                for _ in range(10):
                    L.run_cycle()
                L.inject(F.SeatGate(corner=corner, how=how))
                for _ in range(120):
                    L.run_cycle()
                self.assertGreater(sum(not r["seated"] for r in L.records), 5)
                self.assertEqual([r for r in L.records if not r["seated"] and not r["flag"]], [], (corner, how))

    def test_flag_setting_masks_one_gate_keeps_running_and_stops_on_a_second(self):
        """Property: set to 'flag', the cell runs on three gates and never on two. Whatever the first
        and the second bad gate do, every cycle after the second is flagged until a person has been asked."""
        for how in ("high", "low", "missing"):
            for second in ("high", "low", "missing"):
                tag = f"{how}, then {second}"
                L = Line(2003, on_gate_fault="flag")
                for _ in range(5):
                    L.run_cycle()
                L.inject(F.SeatGate(corner=1, how=how))
                for _ in range(30):
                    L.run_cycle()
                self.assertEqual([n for _, n in L.notices], [f"seat_gate_masked:1:{how}"], tag)
                self.assertFalse(any(e["hands"] for e in L.recoveries), tag)
                self.assertTrue(all(r["seated"] and not r["flag"] for r in L.records[-25:]), tag)
                L.cell.faults.seat_corner[3] = second                       # a second gate goes
                n0 = len(L.records)
                for _ in range(40):
                    L.run_cycle()
                    if any(e["hands"] for e in L.recoveries):
                        break
                after = L.records[n0:]
                self.assertTrue(any(e["hands"] for e in L.recoveries), tag)
                self.assertTrue(all(r["flag"] for r in after), tag)
                self.assertTrue(all(r["masked"] == [1] for r in after), tag)    # the second gate is never masked
                self.assertEqual(len(L.notices), 1, tag)
                if second != "low":
                    self.assertEqual(after[0]["hold"], "seat_precheck" if second == "high" else "seat_gate_missing", tag)

    # ---------------------------------------------------------------- three reads
    def test_part_that_leaves_with_the_tool_is_caught_only_by_the_read_with_the_tool_away(self):
        """Property: with the third read no cycle ends unseated and unflagged however often a seated part
        clings to the cup; with the last read taken tool-down, every such cycle is silent."""
        for away, cling in ((True, 0.3), (False, 0.3), (True, 0.0)):
            L = Line(2003, away_read=away, ladder=False)
            L.cell.faults.cling_p = cling
            for _ in range(200):
                L.run_cycle()
            failed = [r for r in L.records if not r["seated"]]
            silent = [r for r in failed if not r["flag"]]
            if cling and away:
                self.assertGreater(len(failed), 30)
                self.assertEqual(silent, [])
                self.assertTrue(all(r["reasons"] == ["not_seated_after_release"] for r in failed))
            elif cling:
                self.assertGreater(len(silent), 30)
            else:
                self.assertEqual(sum(r["flag"] for r in L.records), 0)     # the added read stops no good cycle

    def test_release_interlock_leaves_the_nest_empty_after_a_failed_insertion(self):
        """Every failed insertion that is released lies across the nest in this cell (block_p = 1). With
        the interlock the release is refused, the part goes to the reject place, and nobody is needed."""
        for interlock in (True, False):
            L = Line(2003, stack="calibrated", interlock=interlock)
            L.cell.faults.block_p = 1.0
            blocked = 0
            for _ in range(120):
                L.run_cycle()
                blocked += L.cell.blocked or any(e["action"] == "clear_nest" and e["cycle"] == L.cycle - 1 and
                                                 "seat_precheck" in e["reasons"] for e in L.recoveries[-1:])
            self.assertGreater(sum(not r["seated"] for r in L.records), 5)
            self.assertEqual([r for r in L.records if not r["seated"] and not r["flag"]], [])
            if interlock:
                self.assertEqual(blocked, 0)
                self.assertTrue(any(r["interlock"] for r in L.records))
                self.assertNotIn("seat_precheck", [x for r in L.records for x in r["reasons"]])
            else:
                self.assertGreater(blocked, 0)
                self.assertIn("seat_precheck", [x for r in L.records for x in r["reasons"]])

    def test_release_interlock_holds_one_cycle_and_the_next_runs_with_nobody_clearing_it(self):
        """The interlock refuses one release; it does not stop the cell. No ladder here, so nothing
        outside the runtime lifts the hold: one refused release per interlock, and no cycle lost after it."""
        L = Line(2003, stack="calibrated", ladder=False)
        for _ in range(120):
            L.run_cycle()
        interlocks = sum(r["interlock"] for r in L.records)
        self.assertGreater(interlocks, 5)
        self.assertEqual(L.rt.counts.get("supervisor_hold", 0), interlocks)
        self.assertNotIn("incomplete", [x for r in L.records for x in r["reasons"]])
        first = next(i for i, r in enumerate(L.records) if r["interlock"])
        self.assertTrue(any(r["seated"] and not r["flag"] for r in L.records[first + 1:]))

    def test_running_on_three_gates_misses_only_a_lift_at_the_masked_corner(self):
        """The limit of the 'flag' setting, as a property: every silent failure on three gates is a bad
        seat whose one lifted corner is the masked gate; with the cell set to stop there is none."""
        for mode in ("flag", "stop"):
            L = Line(2003, stack="calibrated", on_gate_fault=mode, ladder=(mode == "stop"), tilt_share=1.0)
            L.inject(F.SeatGate(corner=1, how="high"))
            silent = lifted_at_mask = 0
            for _ in range(400):
                r = L.run_cycle()
                L.rt.clear_hold() if mode == "flag" else None
                if not r["seated"] and not r["flag"]:
                    silent += 1
                    lifted_at_mask += (L.cell.lifted == (1,))
            self.assertEqual(silent, lifted_at_mask, mode)
            (self.assertGreater if mode == "flag" else self.assertEqual)(silent, 0, mode)

    # ---------------------------------------------------------------- freshness tied to an event
    def test_old_look_at_an_unchanged_nest_runs_under_event_freshness_and_is_refused_by_the_clock(self):
        for mode, good in (("event", True), ("clock", False)):
            L = Line(2003, freshness=mode)
            f = L.inject(F.StaleFrame(p=1.0))
            for _ in range(10):
                L.run_cycle()
            self.assertEqual(all(r["seated"] and not r["flag"] for r in L.records), good, mode)
            self.assertEqual(L.rt.counts.get("stale_observation", 0), 0 if good else 10, mode)

    def test_look_taken_before_the_part_arrived_is_refused_under_event_freshness_only(self):
        for mode, refused in (("event", 10), ("clock", 0)):
            L = Line(2003, freshness=mode, ladder=False)
            L.run_cycle()                                                   # time must have passed for a look to be early
            L.inject(F.EarlyLook(p=1.0))
            for _ in range(10):
                L.run_cycle()
            self.assertEqual(L.rt.counts.get("stale_observation", 0), refused, mode)
            self.assertEqual([r for r in L.records if not r["seated"] and not r["flag"]], [], mode)

    def test_runtime_dates_each_nest_by_its_own_grasp_and_release(self):
        """A look at a nest taken before the runtime itself last emptied or filled it is void, with no
        event line from outside. Both looks are young enough to pass the clock rule."""
        # destination: the look at the nest is from before the last cycle's release into it
        L = Line(2003)
        rec = L.run_cycle()
        fresh = L.policy.requests(L.cycle, L.now, L.frame + 1)
        L.rt.scene_event("src", L.now)                                      # the next part arrived
        L.sup.heartbeat(L.now)
        g = L.rt.submit(L._seal(fresh[0]), L.now + 10)
        self.assertEqual(g.outcome, "done")
        t = L.now + 10 + g.duration_ms
        L.sup.heartbeat(t)
        old_place = dict(fresh[1], observed_at_ms=rec["t0"] + 20)
        self.assertLess(t - old_place["observed_at_ms"], old_place["deadline_ms"])
        self.assertEqual(L.rt.submit(L._seal(old_place), t).reason, "stale_observation")
        # source: no part-arrived event, and the look is from before the last grasp emptied the nest
        L = Line(2003)
        rec = L.run_cycle()
        old_grasp = dict(L.policy.requests(L.cycle, rec["t0"] + 5, L.frame + 1)[0])
        L.sup.heartbeat(L.now)
        self.assertLess(L.now + 10 - old_grasp["observed_at_ms"], old_grasp["deadline_ms"])
        self.assertEqual(L.rt.submit(L._seal(old_grasp), L.now + 10).reason, "stale_observation")
        self.assertEqual(L.cell.at, "home")

    def test_look_older_than_the_last_recovery_is_refused(self):
        """A person's hands or the arm were in the cell after the look: the look is void."""
        L = Line(2003)
        L.run_cycle()
        reqs = L.policy.requests(L.cycle, L.now, L.frame + 1)
        L.now += 100
        L.rt.scene_event("cell", L.now)
        L.sup.heartbeat(L.now)
        self.assertEqual(L.rt.submit(L._seal(reqs[0]), L.now + 10).reason, "stale_observation")

    def test_observation_from_the_future_is_refused_under_both_rules(self):
        for mode in ("event", "clock"):
            L = Line(2003, freshness=mode)
            L.sup.heartbeat(0)
            r = dict(L.policy.requests(0, 0, 1)[0], observed_at_ms=10 ** 9)
            self.assertEqual(L.rt.submit(L._seal(r), 10).reason, "stale_observation", mode)
            self.assertEqual(L.cell.at, "home")

    def test_backstop_age_still_refuses_under_event_freshness(self):
        L = Line(2003)
        L.inject(F.StaleFrame(p=1.0, age_ms=400_000))
        L.run_cycle()
        self.assertEqual(L.rt.counts.get("stale_observation", 0), 1)

    def test_software_faults_clear_without_hands(self):
        for fault in (F.PolicyDead(), F.StaleCalibration(), F.FrozenCamera(), F.RuntimeHung(), F.SupervisorDead(),
                      F.LogFull(), F.BadUpdate(mm=4.0), F.BadUpdate(mm=2.0)):
            L, f, ep = self.episode(fault, run=120)
            self.assertNotIn(f, L.faults, f.name)
            self.assertFalse(any(e["hands"] for e in L.recoveries), f.name)
            self.assertTrue(all(r["seated"] and not r["flag"] for r in ep[-30:]), f.name)

    def test_base_shift_is_caught_by_the_trend_and_fixed_by_recalibration(self):
        L, f, ep = self.episode(F.Physical(fault="base_shift_mm", value=1.0), run=150)
        self.assertIn("recalibrate", [e["rung"] for e in L.recoveries])
        self.assertLessEqual(sum(not r["seated"] for r in ep), 3)

    def test_device_faults_need_hands_and_say_so(self):
        for fault in (F.Stall(), F.SeatStuck(how="missing"), F.GripperMissing()):
            L, f, ep = self.episode(fault)
            self.assertTrue(any(e["hands"] for e in L.recoveries), f.name)

    def test_same_seed_same_run(self):
        a, _, _ = self.episode(F.DropPacket(p=0.2))
        b, _, _ = self.episode(F.DropPacket(p=0.2))
        self.assertEqual(a.records, b.records)
        self.assertEqual(a.log.prev, b.log.prev)


class Counting(unittest.TestCase):
    def test_good_part_rejected_on_a_false_sensor_read_counts_as_a_good_cycle_stopped(self):
        """The combined-figure count (campaign section 5). With the release interlock a false 'empty'
        sends a good part to the reject place; that is a false alarm, not a failure caught."""
        from cellrt.campaign import _r3
        _, out = _r3((2003, "calibrated_uni", 0.5, None, 0.0, 0.2, 150))
        self.assertGreater(out["sup_g"], 15)                 # about 30 false reads in 150 cycles
        self.assertLessEqual(out["failed"], 2)
        self.assertEqual(out["good"] + out["failed"], 150)


class LogAndReplay(unittest.TestCase):
    def test_chain_detects_edit_removal_and_reorder(self):
        L = Line(2000)
        for _ in range(5):
            L.run_cycle()
        lines = list(L.log.lines)
        self.assertTrue(verify(lines)[0])
        self.assertFalse(verify(lines[:3] + lines[4:])[0])
        self.assertFalse(verify(lines[:3] + [lines[4], lines[3]] + lines[5:])[0])
        e = json.loads(lines[10])
        e["t"] += 1
        self.assertFalse(verify(lines[:10] + [json.dumps(e, sort_keys=True, separators=(",", ":"))] + lines[11:])[0])

    def test_owner_edit_passes_the_chain_and_fails_against_an_anchor_held_elsewhere(self):
        """The chain alone cannot show an edit by whoever holds the file."""
        L = Line(2003, stack="calibrated")
        for _ in range(40):
            L.run_cycle()
        anchor = L.log.anchor()
        for _ in range(5):
            L.run_cycle()
        self.assertTrue(verify(L.log.lines, [anchor])[0])
        forged, prev, changed = [], "0" * 64, 0
        for line in L.log.lines:                                             # clear every flag, recompute every hash
            e = json.loads(line)
            if e["k"] == "verdict" and e["d"]["flag"]:
                e["d"]["flag"], changed = False, changed + 1
            e["p"] = prev
            body = json.dumps(e, sort_keys=True, separators=(",", ":"))
            prev = __import__("hashlib").sha256(body.encode()).hexdigest()
            forged.append(body)
        self.assertGreater(changed, 0)
        self.assertTrue(verify(forged)[0])
        self.assertFalse(verify(forged, [anchor])[0])
        self.assertFalse(verify(L.log.lines[:anchor["n"] - 1], [anchor])[0])  # anchored lines removed

    def test_no_image_and_no_key_in_the_log(self):
        L = Line(2000)
        for _ in range(3):
            L.run_cycle()
        blob = "\n".join(L.log.lines)
        self.assertNotIn(KEY.decode(), blob)
        self.assertNotIn("image", blob)

    def test_replay_reproduces_every_reply(self):
        L = Line(2005)
        L.inject(F.DropPacket(p=0.2, step=None))
        L.inject(F.ModelOffset(p=0.2, mm=4.0))
        L.inject(F.Malformed(p=0.2, variant="mix"))
        L.inject(F.EarlyLook(p=0.2))                                        # refusals that depend on scene events
        L.inject(F.StaleFrame(p=0.1))
        for _ in range(60):
            L.run_cycle()
        self.assertGreater(L.rt.counts.get("stale_observation", 0), 3)
        out = replay(L.log.lines, {KID: KEY})
        self.assertTrue(out["chain_ok"])
        self.assertGreater(out["decisions"], 100)
        self.assertEqual(out["mismatches"], [])

    def test_replay_notices_a_changed_rule(self):
        L = Line(2005)
        L.inject(F.ModelOffset(p=0.5, mm=2.0))
        for _ in range(30):
            L.run_cycle()
        out = replay(L.log.lines, {KID: KEY}, limits=dataclasses.replace(Limits(), nest_tol_mm=1.0))
        self.assertTrue(out["mismatches"])


class Release(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.s = R.Slots(self.d, b"vendor-key")
        self.s.factory(R.make_bundle(b"vendor-key", R.Manifest(1, "transfer", "1.0.0", "cal-1", "cellrt-0.1", "cfg-0"),
                                     {"bias": [0, 0]}))

    def bundle(self, release, key=b"vendor-key", version="1.1.0"):
        return R.make_bundle(key, R.Manifest(release, "transfer", version, "cal-1", "cellrt-0.1", "cfg-0"), {"bias": [0, 0]})

    def test_install_activate_rollback(self):
        slot = self.s.install(self.bundle(2))
        self.assertEqual(self.s.boot()[1].release, 1)          # installing does not switch
        self.s.activate(slot)
        self.assertEqual(self.s.boot()[1].release, 2)
        self.s.rollback()
        self.assertEqual(self.s.boot()[1].release, 1)

    def test_refuses_wrong_key_corruption_and_downgrade(self):
        with self.assertRaises(R.BadBundle):
            self.s.install(self.bundle(2, key=b"someone-else"))
        raw = bytearray(self.bundle(2))
        raw[len(raw) // 2] ^= 1
        with self.assertRaises(R.BadBundle):
            self.s.install(bytes(raw))
        self.s.activate(self.s.install(self.bundle(3)))
        for old in (1, 2, 3):
            with self.assertRaises(R.BadBundle):
                self.s.install(self.bundle(old))
        self.assertEqual(self.s.boot()[1].release, 3)

    def test_power_loss_at_any_point_leaves_a_bootable_device(self):
        with self.assertRaises(KeyboardInterrupt):
            self.s.install(self.bundle(2), crash_after="partial")       # torn write under the staged name
        self.assertEqual(self.s.boot()[1].release, 1)
        slot = self.s.install(self.bundle(2))                            # written, pointer not switched
        self.assertEqual(self.s.boot()[1].release, 1)
        self.s.activate(slot)
        with open(os.path.join(self.d, f"slot_{slot}.bundle"), "r+b") as f:   # the active slot rots
            f.seek(20)
            f.write(b"\x00\x00")
        self.assertEqual(self.s.boot()[1].release, 1)                    # falls back to the other slot

    def test_lost_pointer_never_boots_a_rejected_or_a_never_activated_release(self):
        """Release 1 in a; 2 into b, activated; 3 into a, activated, rolled
        back. Whatever then happens to the pointer file, the unit runs release 2."""
        ptr = os.path.join(self.d, "pointer.json")
        self.s.activate(self.s.install(self.bundle(2)))
        self.s.activate(self.s.install(self.bundle(3)))
        self.assertEqual(self.s.boot()[1].release, 3)
        self.assertEqual(self.s.rollback(), "b")
        with open(ptr) as f:
            good = f.read()
        for content in (good, None, "", "{}", '{"active": "c", "previous": null}', "[1]", '{"active": "a"}', "\x00\x00"):
            if content is None:
                os.remove(ptr)
            else:
                with open(ptr, "w") as f:
                    f.write(content)
            self.assertEqual(self.s.boot()[1].release, 2, repr(content))
        with self.assertRaises(R.BadBundle):
            self.s.install(self.bundle(3))                                   # the rejected release does not come back
        staged = self.s.install(self.bundle(4))                              # staged, not activated
        os.remove(ptr)
        self.assertEqual(self.s.boot()[1].release, 2)
        self.s.activate(staged)
        os.remove(ptr)
        self.assertEqual(self.s.boot()[1].release, 4)                        # activated and never rejected: newest wins

    def test_power_loss_inside_a_rollback_never_leaves_the_rejected_release_bootable(self):
        self.s.activate(self.s.install(self.bundle(2)))
        os.replace(os.path.join(self.d, "slot_b.bundle"), os.path.join(self.d, "slot_b.rejected"))   # first half of rollback
        self.assertEqual(self.s.boot()[1].release, 1)                        # pointer still names b; b cannot boot
        os.remove(os.path.join(self.d, "pointer.json"))
        self.assertEqual(self.s.boot()[1].release, 1)

    def test_shadow_replay_stops_a_shifted_policy_before_it_moves(self):
        m2 = R.Manifest(2, "transfer", "1.1.0", "cal-1", "cellrt-0.1", "cfg-0")
        logged = [dict(obs={}, targets={"grasp": [190.0, -60.0, 30.0], "place": [190.0, 60.0, 30.0]})] * 50

        def policy(bias):
            def f(obs):
                return [dict(GOOD, cycle=0, step=0, kind="grasp", target_xyz_mm=[190.0, -60.0, 30.0], policy_version="1.1.0"),
                        dict(GOOD, cycle=0, policy_version="1.1.0", target_xyz_mm=[190.0 + bias, 60.0, 30.0])]
            return f
        admit = lambda r: admit_static(r, m2)
        self.assertTrue(R.shadow_replay(policy(0.0), logged, admit)["passed"])
        self.assertTrue(R.shadow_replay(policy(0.8), logged, admit)["passed"])
        self.assertFalse(R.shadow_replay(policy(0.8), logged, admit, max_shift_mm=0.5)["passed"])
        self.assertFalse(R.shadow_replay(policy(2.0), logged, admit)["passed"])
        out = R.shadow_replay(policy(4.0), logged, admit)
        self.assertEqual((out["passed"], out["reasons"]), (False, {"not_a_nest": 50}))


if __name__ == "__main__":
    unittest.main()
