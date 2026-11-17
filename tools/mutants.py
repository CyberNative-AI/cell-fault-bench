"""Each supervisor, runtime and release rule, removed one at a time; the test that names the rule
must then fail. A mutant that survives means the test does not hold the rule.

    python3 tools/mutants.py > results/mutants.json

Works on a copy in a temporary directory. Nothing in the
repository is changed.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

SEC = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

# (name, file, text to find exactly once, replacement, -k patterns of the tests that must fail)
MUTANTS = [
    ("cycle record carries one seat bit, not four", "cellrt/supervisor.py",
     'at="pre", seat=_bits(pre))', 'at="pre", seat=_bits(pre[:1]))',
     ["test_cycle_record_carries_four_seat_bits"]),
    ("one corner high before a load is treated as a part", "cellrt/supervisor.py",
     "if len(high) == 1 and len(live) == len(pre):", "if False:",
     ["test_one_gate_high_before_a_load_is_named"]),
    ("same corner low is never called a gate", "cellrt/supervisor.py",
     "GATE_LOW_STREAK = 3 ", "GATE_LOW_STREAK = 10 ** 9 ",
     ["test_one_gate_low_is_named_on_the_third_cycle"]),
    ("flag setting masks a second bad gate too", "cellrt/supervisor.py",
     'if self.on_gate_fault == "flag" and not self.masked:', 'if self.on_gate_fault == "flag":',
     ["test_flag_setting_masks_one_gate_keeps_running_and_stops_on_a_second"]),
    ("a gate that gives no reading is not named by the supervisor", "cellrt/line.py",
     'GATE_NAMED = {"seat_gate_stuck", "seat_gate_missing"}', 'GATE_NAMED = {"seat_gate_stuck"}',
     ["test_one_gate_high_before_a_load_is_named"]),
    ("a good part rejected on a false sensor read is counted as a failure caught", "cellrt/campaign.py",
     'good = r["seated"] or r["good_blind"]', 'good = r["seated"]',
     ["test_good_part_rejected_on_a_false_sensor_read"]),
    ("no seat read with the tool away", "cellrt/supervisor.py",
     'self._read_seat("away", now_ms)', "pass",
     ["test_part_that_leaves_with_the_tool"]),
    ("gates open with the tool down do not refuse the release", "cellrt/supervisor.py",
     'if self._read_seat("down", now_ms) and self.interlock and not v.hold:',
     'if self._read_seat("down", now_ms) and False:',
     ["test_release_interlock_leaves_the_nest_empty"]),
    ("a hold for one cycle is never lifted", "cellrt/runtime.py",
     "if self.hold_cycle is not None and cycle == self.hold_cycle:", "if False:",
     ["test_release_interlock_holds_one_cycle"]),
    ("a look taken before the nest changed is accepted", "cellrt/runtime.py",
     'elif req.observed_at_ms < self.scene_ms["src" if req.kind == "grasp" else "dst"]:', "elif False:",
     ["test_look_taken_before_the_part_arrived", "test_look_older_than_the_last_recovery"]),
    ("the runtime's own grasp and release do not date the nests", "cellrt/runtime.py",
     'if req.kind in ("grasp", "release"):', "if False:",
     ["test_runtime_dates_each_nest_by_its_own_grasp_and_release"]),
    ("an observation dated after now is accepted", "cellrt/runtime.py",
     "if age < 0:", "if False:",
     ["test_observation_from_the_future"]),
    ("no backstop age under event freshness", "cellrt/runtime.py",
     "elif age > self.lim.obs_max_age_ms:", "elif False:",
     ["test_backstop_age_still_refuses"]),
    ("rollback leaves the rejected bundle under its bootable name", "cellrt/release.py",
     'os.replace(self._p(f"slot_{cur}.bundle"), self._p(f"slot_{cur}.rejected"))', "pass",
     ["test_lost_pointer_never_boots", "test_power_loss_inside_a_rollback"]),
    ("an installed bundle is bootable before it is activated", "cellrt/release.py",
     'self._write_atomic(f"slot_{target}.staged", raw)',
     'self._write_atomic(f"slot_{target}.staged", raw); self._write_atomic(f"slot_{target}.bundle", raw)',
     ["test_lost_pointer_never_boots"]),
    ("a pointer naming an unknown slot is believed", "cellrt/release.py",
     'or p["active"] not in ("a", "b")', "or False",
     ["test_lost_pointer_never_boots"]),
    ("a rejected release number can be installed again", "cellrt/release.py",
     'floor = max([cur.release] + [r[0].release for r in (self.read(s, "rejected") for s in ("a", "b")) if r])',
     "floor = cur.release",
     ["test_lost_pointer_never_boots", "test_install_activate_rollback", "test_refuses_wrong_key"]),
    ("a consistent chain that is not the anchored one passes", "cellrt/eventlog.py",
     "if want.get(i + 1, prev) != prev:", "if False:",
     ["test_owner_edit_passes_the_chain_and_fails_against_an_anchor"]),
    ("a log that lost anchored lines passes", "cellrt/eventlog.py",
     "if any(n > len(lines) for n in want):", "if False:",
     ["test_owner_edit_passes_the_chain_and_fails_against_an_anchor"]),
]


def run(tmp, patterns):
    cmd = [sys.executable, "-m", "unittest", "discover", "-s", "tests"]
    for p in patterns:
        cmd += ["-k", p]
    r = subprocess.run(cmd, cwd=tmp, capture_output=True, text=True, timeout=900)
    tail = r.stderr.strip().splitlines()[-1] if r.stderr.strip() else ""
    ran = next((ln for ln in r.stderr.splitlines() if ln.startswith("Ran ")), "")
    return r.returncode, ran, tail


def main():
    out = []
    with tempfile.TemporaryDirectory() as tmp:
        shutil.copytree(SEC, tmp, dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns(".git", "results", ".venv", "__pycache__"))
        for name, rel, find, repl, tests in MUTANTS:
            path = os.path.join(tmp, rel)
            with open(path) as f:
                src = f.read()
            row = dict(mutant=name, file=rel, tests=tests)
            if src.count(find) != 1:
                row.update(result="NOT RUN", why=f"text found {src.count(find)} times, wanted 1")
                out.append(row)
                continue
            base = run(tmp, tests)                       # the named tests pass on the unchanged copy
            with open(path, "w") as f:
                f.write(src.replace(find, repl))
            try:
                mut = run(tmp, tests)
            finally:
                with open(path, "w") as f:
                    f.write(src)
            ok = base[0] == 0 and base[1] != "Ran 0 tests" and not base[1].startswith("Ran 0 ") and mut[0] != 0
            row.update(result="killed" if ok else "SURVIVED" if base[0] == 0 else "BASE FAILED",
                       unchanged=f"{base[1]}: {base[2]}", mutated=f"{mut[1]}: {mut[2]}")
            out.append(row)
            print(row["result"], name, file=sys.stderr, flush=True)
    doc = dict(mutants=len(out), killed=sum(r["result"] == "killed" for r in out),
               survived=[r["mutant"] for r in out if r["result"] == "SURVIVED"],
               not_run=[r["mutant"] for r in out if r["result"] in ("NOT RUN", "BASE FAILED")], rows=out)
    json.dump(doc, sys.stdout, indent=1)
    print()


if __name__ == "__main__":
    main()
