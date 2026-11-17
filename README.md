# cell-fault-bench

A fault bench for the software of a fixtured robot cell: a cell that picks a part from one nest and
places it into another, with a seat sensor in the destination nest. The bench injects 45 classes of
fault into a running cell (wire, forged and malformed commands, model errors, stale camera looks,
dead and hung processes, bad updates, broken sensors, stalled motors, mechanical drift) and counts,
on the same cycles, three things:

- **failures caught**: parts that did not seat, would not have seated with no checks either, and
  that the checks flagged;
- **silent failures**: parts that did not seat and that nobody flagged;
- **good cycles stopped**: parts that seated, or would have, that the checks flagged anyway.

The third count is made with a blind twin: every cycle is replayed with the same bytes through a
controller with no checks, so a flagged cycle that would have seated without the checks counts as a
false alarm, not as a catch.

The worked example is a simulated SO-101 arm (`cellsim/`) placing a part into a nest with 3 mm of
lead-in, under a reference controller (`cellrt/`). **Every bench result in this README is from that
simulation. None is a physical measurement.** The model's parameter table (Example cell parameters)
cites its sources; one of them is a bench test of a physical servo.

## Run it

Tested with Python 3.14.4 and numpy 2.3.5 on Linux. A clean install in a new venv with Python
3.11.2 and numpy 2.4.6 also passed the test suite.

```sh
git clone https://github.com/CyberNative-AI/cell-fault-bench
cd cell-fault-bench
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
python3 -m unittest discover -s tests
OMP_NUM_THREADS=1 python3 -m cellrt.campaign --procs 5 --out results/model-c3
```

The test suite has 55 tests and took about 80 s on our host. The campaign prints the unit seeds,
the cycle count and two tables when it finishes, and writes `results/model-c3/campaign.json`. On
our host (5 processes) it took 731 s. Add `--quick` for a
one-minute run on 4 units. `python3 -m cellrt.scorecard results/model-c3/campaign.json --by-class`
prints one row per fault class from the saved file.

Its output on our host:

```text
Simulated cell, model arm, 3.0 mm lead-in. Unit seeds 2000-2019 (20 units). Python 3.14.4. Campaign wall time 730.8 s.
394,000 cycles in 45 fault classes (51 cases: a few classes are rerun with one check switched off or with a clock rule, shown in brackets with --by-class).

| faults | cycles | failures | caught | silent | silent per 10,000 | good cycles | good stopped |
|---|---|---|---|---|---|---|---|
| commands and messages (7) | 70,000 | 1,742 | 1,742 | 0 | 0.0 | 68,258 | 290 |
| policy (model) errors (7) | 110,000 | 3,670 | 3,670 | 0 | 0.0 | 106,330 | 264 |
| stale or early camera looks (2) | 40,000 | 24 | 24 | 0 | 0.0 | 39,976 | 991 |
| processes, calibration, log (6) | 36,000 | 127 | 127 | 0 | 0.0 | 35,873 | 199 |
| bad software updates (3) | 18,000 | 126 | 126 | 0 | 0.0 | 17,874 | 7 |
| seat and gripper sensors (7) | 42,000 | 8 | 8 | 0 | 0.0 | 41,992 | 471 |
| motors and the nest (3) | 18,000 | 125 | 125 | 0 | 0.0 | 17,875 | 0 |
| mechanics and camera drift (10) | 60,000 | 1,181 | 1,181 | 0 | 0.0 | 58,819 | 0 |
| **all fault cases** | 394,000 | 7,003 | 7,003 | 0 | 0.0 | 386,997 | 2,222 |

Good cycles no fault acted on (untouched cycles of the transient cases, no variants, plus the fault-free runs with a truthful seat sensor): 180,674, of which stopped: 0.

| seat sensor and cling cases (no other fault) | cycles | failures | caught | silent | silent per 10,000 | good cycles | good stopped |
|---|---|---|---|---|---|---|---|
| fault-free place stack (about 12% of placements fail on the model arm); truthful seat sensor | 10,000 | 1,195 | 1,195 | 0 | 0.0 | 8,805 | 0 (0.000%) |
| same stack; sensor reads "seated" on 1% of unseated parts, "empty" on 0.1% of seated ones | 10,000 | 1,195 | 1,178 | 17 | 17.0 | 8,805 | 11 (0.125%) |
| same stack; the same at 5% and 0.5% | 10,000 | 1,195 | 1,142 | 53 | 53.0 | 8,805 | 56 (0.636%) |
| tighter stack (about 1 in 10,000 fail on the model arm); 1% of seated parts leave with the tool; last gate read with the tool down | 10,000 | 82 | 1 | 81 | 81.0 | 9,918 | 0 (0.000%) |
| the same; last gate read with the tool away | 10,000 | 82 | 82 | 0 | 0.0 | 9,918 | 0 (0.000%) |
```

"Good stopped" in the fault cases is the price of the checks: a malformed or stale request refused on
a cycle that would have seated anyway, a model target 3 mm off that is refused though the lead-in
would have saved the part, and the cycles on which a broken sensor or process is found and the cell
holds (`--by-class` shows each class). In the second table it is a seat sensor that reads "empty" on
a seated part, which sends a good part to the reject place.

### What the tables say, and what they do not

- **With a truthful seat sensor, no failure got through silently: 0 in 394,000 fault cycles, and 0
  of 180,674 untouched good cycles were stopped.** That holds for a cell in which a released part
  stays where it was put.
- **A part that leaves with the tool** (row 4 of the second table) got through 81 times in 10,000
  when the last seat read is taken with the tool still down. A third read of the same gates once the
  tool is away (row 5, the controller's default) catches all of them.
- **Detection is what the fixture sensor sees at the moments it is read, for lifts above its
  acceptance.** With a sensor that reads "seated" on 1% of unseated parts the checks caught 1,178 of
  1,195 failures; at 5%, 1,142. Software neither adds to that nor repairs it. The number to go and
  measure is your sensor's, on your fixture.
- **Two simulated arm profiles built from public recordings gave the same result: no silent failure
  and no untouched good cycle stopped.** They fail far more placements than the model arm: 60,900
  (p50, 3 mm) and 139,224 (p90, 5 mm) of 394,000 fault cycles, against 7,003. `cellsim/arms.py`
  sets each joint's play and settle scatter to that joint's own median ("p50") or its own 90th
  percentile ("p90") in the `joints` table of the public
  [holdcheck corpus summary](https://github.com/CyberNative-AI/holdcheck/blob/be1ebd42/holdcheck/corpus_summary.json).
  The corpus measured 1,574 public LeRobot datasets across several robot families; the `joints`
  table uses only its SO-100 and SO-101 families, 371 arm aggregates (226 and 145), each one
  contributor's datasets of one family. An aggregate is not one physical arm. A profile is
  not a real arm either, because the joints of one arm are not equally loose. With the p50 profile
  at 3 mm of lead-in and the p90 profile at 5 mm, 0 of 394,000 fault cycles were silent and 0
  untouched good cycles were stopped (151,525 and 114,086 good cycles). Reproduce with
  `--arm p50 --c-mm 3` (1,364 s on our host) and `--arm p90 --c-mm 5` (1,577 s).

## Run the bench against your cell

The bench talks to a cell through one object, the **device**. `cellrt/plant.py` `SimCell` is the
example cell's device; `examples/toy_cell.py` is a 140-line device written without the arm model,
which is the place to start an adapter for your own cell simulation.

```sh
OMP_NUM_THREADS=1 python3 -m cellrt.campaign --device examples.toy_cell:ToyCell --out results/toy
```

`--device module:factory` names a callable `factory(unit_seed, c_mm, seed)` that returns a device.
A device has:

| member | what it does |
|---|---|
| `begin_cycle()` | a new part waits at the source nest; the last one has left the destination, unless a part lies across the nest |
| `do(kind, target_xy)` | the only way to move: `"grasp"`, `"place"` (lower into the nest, vacuum on), `"release"`, `"home"`, `"reject"`. Returns `dict(moved, torque_peak, duration_ms)`. Targets are task-frame mm; the nests are `cellrt.plant.SRC_NEST` and `DST_NEST` |
| `tap()` | the supervisor's read-only wires: `seat()` four corner gates (True = that corner is down, None = no reading), `gripper_holds()`, `verify_obs_mm()` (one overhead look before release, or None) |
| `truth()` | `dict(seated=..., miss_mm=...)`: what really happened. Only the bench reads it |
| `twin()` | an independent copy at the start of a cycle, for the blind replay. Moving the twin must not move the device or use its random stream |
| `recalibrate()` | automatic recalibration; may do nothing |
| `hands_clear()` | a person clears a part lying across the nest |
| `faults` | a `cellrt.plant.DeviceFaults`. The bench sets its fields; the device honours the ones it lists in `SUPPORTS` |
| `c` | the nest's lead-in, mm |

Faults on the wire, in the controller, in the processes and in updates are applied by the bench to
any device. Sensor and motor faults (`seat_stuck`, `seat_corner`, `seat_p_miss`, `seat_p_false`,
`gripper_stuck`, `stall`, `torque_spike`, `blocked`) are fields your device honours, because only
it knows what a stuck gate or a stalled axis looks like in your cell. Classes that need a field
your device does not list, and the mechanical classes that need an arm model (`physics`), are not
run; the scorecard lists them under "Not run on this device". The sensor-misread rows run when the
device supports both `seat_p_miss` and `seat_p_false`.

**What is scored is your cell under the reference controller in `cellrt/`, not your own
controller.** Scoring your own controller needs a controller interface, and the process faults
(hang, dead supervisor, full log, lost heartbeat) reach into the reference runtime today. If you
want that, say so on the controller-seam issue.

### Two modules to take into your own controller

Both use only the Python standard library and import nothing else from this repo.

- **`cellrt/supervisor.py`, the seat-transition check.** Four corner gates, each tested every cycle
  by its own transition: all four empty before the load, all four closed with the tool down and the
  vacuum still on, all four closed again once the tool is away. A gate stuck high is found at the
  first pre-load read; one stuck low after three placements the other three call good. With a seat
  signal stuck high, this check stopped the cell before the first part; with it removed, 632 failures
  in 6,000 cycles got through silently (`python3 -m cellrt.report results/model-c3/campaign.json`,
  section 4, which also shows what each other check buys).
- **`cellrt/release.py`, the release gate.** Signed release bundles, two slots with one atomic
  pointer switch, refusal of a downgrade, a shadow replay of the unit's own logged cycles before the
  switch, and rollback. Boot falls back to a slot that verifies in the cases the tests simulate in
  files: a torn write of the staged slot, an active slot that fails its hash, a missing or malformed
  pointer, and a rollback cut off half way. A real power cut and storage that reorders writes are
  not tested. The pointer is written with a file fsync and a rename, with no fsync of the
  directory, and boot does not rewrite a lost pointer.

## Counting

Every cycle started is one of:

| | seated | flagged | blind twin |
|---|---|---|---|
| A good | yes | no | |
| B false alarm | yes | yes | |
| C false alarm | no | yes | would have seated |
| D caught failure | no | yes | would have failed |
| E silent failure | no | no | |

failures = D + E; caught = D; silent = E; good cycles = A + B + C; good stopped = B + C. A
transient fault case counts both the cycles its fault hit and the ones it did not; a persistent case
counts every cycle of every episode. The definitions are in `cellrt/campaign.py` and
`cellrt/scorecard.py`.

## Limits

- **Simulation of a stated model.** `cellsim` knows the lateral miss and the tool's tilt; it has no
  contact model. "Hangs with one edge up" is a mapping in `cellrt/plant.py`, not physics.
- **Base-shift sizes.** The example cell's scorer adds half the pick error to the place error where
  physically it subtracts (marked in `cellrt/plant.py`). In a paired study of the cell model alone
  (no fault bench), a moved base showed in the verify reading at about 1.45 times the bump with the
  sign used here and about 0.48 times with the physical sign. On that study, each base-shift label
  in the campaign stands for a bump roughly three times larger: the "1 mm" row is about 3 mm. The
  scale rests on the assumed carry of 0.5 (see Example cell parameters). Every campaign result in
  this README, including which check catches what and the zero silent failures, is for the sign
  shipped here. The campaign was not run with the physical sign.
- **The recovery ladder's escalation rule is fixed** (3 failed seats in a row, or 4 in 60 cycles).
  On a real hobby-servo arm, which fails some seats with no fault present, it calls for help far more
  often than on the modelled arm. See the known-limit issue.
- **Keys.** Commands and release bundles are signed with HMAC and a shared key, as a stand-in. A
  product signs releases with an offline private key and gives each device its own key. The event
  log holds numbers derived from camera images, never the images.
- **The supervisor here is a separate object in the same process.** In a product it belongs on a
  separate small controller with its own power and its own wire to the motor enable. Nothing in
  this repo is a safety function, and no statement about a person's safety may rest on it.

## Example cell parameters

`cellsim/cell.py` and `cellsim/so101.py` are a copy of a quasi-static research model that was frozen
before this bench was built (sha256 of the frozen files: `cell.py`
`e6369b4564413617bb1c40dfc067382175c512c915fa41355d3bc4763c6aadac`, `so101.py`
`b0932ac95a2cc811f6ee9e680efea6f3552cfde885823585d8e4d7b7ecbffc34`). It is bundled with its
author's agreement. The copies here differ from the frozen files in three ways:

- **Cut down.** `cell.py` keeps only what the bench uses. These whole definitions were removed:
  `ctrl_nominal`, `C_GRID_MM`, `Task`, `run_transfers`, `success_at`, `success_curve`, `c_at`,
  `wilson`. The bench's own task constants and scoring are in `cellrt/plant.py`.
- **Comments.** Comments and docstrings point to this README and to `tests/` instead of documents
  that are not published.
- **The p50 and p90 profiles (new code).** `Mech.play_deg` and `Mech.scatter_deg`,
  `MEASURED_BASE_LASH`, `half_width_rad`, a branch at the top of `Unit._lash` (every joint is a play
  operator of its own width plus Gaussian settle scatter; the loaded/free split below is not used),
  and in `move` an approach distance of three widths of the widest joint's play. With `play_deg`
  unset, the default and the "model arm" in this README, none of the added code runs.
  `tests/test_arms.py` checks that the default arm is unchanged. `Unit.command_for` (used to
  generate demonstrations; the bench does not call it) models the default arm only and raises
  `NotImplementedError` on a p50 or p90 profile. The profiles take the table's play and free-hold
  scatter but not its `bias_deg` column, which is large on the loaded joints (p50 0.91° on
  shoulder_lift, 1.30° on elbow_flex); on these profiles gravity acts only through the model's servo
  sag (`tau / kp` in `Unit.settle`).

Defaults of the model arm. Everything except the kinematics, masses and joint ranges (read from
`third_party/so101/so101.xml`) is a parameter:

| parameter | default | where | basis |
|---|---|---|---|
| servo position gain | 17.8 N·m/rad | `Mech.kp` | `third_party/so101/joints_properties.upstream.xml`; Menagerie's `so101.xml` uses 998.22. Neither is a measured STS3215 stiffness |
| gear lash | ±0.5° | `Mech.lash_deg` | the `backlash` class in both upstream files; one vendor bench test of a single STS3215 reports about 0.87 to 1° ([SO-ARM100 issue 134](https://github.com/TheRobotStudio/SO-ARM100/issues/134)) |
| lash type | play: the link lags the motor inside the gap, 20% random part | `Mech.lash_model`, `lash_jitter` | assumption |
| lash take-up threshold | 0.052 N·m of gravity torque | `Mech.preload_tau` | upstream `frictionloss`, used as a threshold (assumption) |
| command resolution | 2π/4096 | `Mech.enc_counts` | STS3215 12-bit encoder |
| joint zero offset, sd per unit | 1.0° | `Mech.offset_sd_deg` | assumption |
| link-origin tolerance, sd per axis | 0.3 mm | `Mech.link_sd_mm` | assumption |
| base mounting error, sd | 2 mm in x and y, 0.5° yaw | `Mech.base_sd_mm`, `base_yaw_sd_deg` | assumption |
| payload at the tool point | 0.02 kg | `Mech.payload` | assumption |
| camera noise per look | 0.5 mm per axis, zero mean | `Cam.sd_mm` | assumption |
| where a loaded joint sits in its lash | uniform over the 10% of the lash nearest the gravity side | `Unit._lash`, `Unit.command_for` (`1.0 - 0.1 * rng.random`) | assumption, no source. In the research model's own runs, changing it moved the result as much as the lash type did (at 1.0° of lash and 3 mm, the best stack fell from 99.83% to 92.53% as it went to the whole lash) |
| tilt the nest tolerates | 4° | `cellrt/plant.py` `TILT_MAX_DEG` | assumption |
| lateral miss the gripper still closes on | 6 mm | `cellrt/plant.py` `GRASP_CAPTURE_MM` | assumption |
| share of the pick error carried as in-hand offset | 0.5 | `cellrt/plant.py` `CARRY` | assumption (its sign: Limits, base-shift sizes) |
| shape of the capture edge | a hard step at the lead-in `c` (3 mm by default) | `cellrt/plant.py` `SimCell.do("place")` | assumption |
| one-direction approach distance | three lash widths on every joint | `move(uni=True)` | assumption |

The last six rows were constants in the code that the research model's first parameter table left
out. Other things a reader who checks the code will find:

- **In the model author's sample, two joints carried the load.** On 2,000 targets with the default
  arm, shoulder lift was loaded on 99% and elbow flex on 100%; pan, wrist flex and wrist roll were
  loaded on none, so on those targets they followed the free-lash model.
- **Sag depends almost entirely on the gain.** In the same 2,000-target sample, at 17.8 N·m/rad the
  median sag was 0.97° on shoulder lift (1.79° at most) and 1.13° on elbow flex; at 998.22 it was
  under 0.04°.
- **The camera has no bias.** It returns world positions plus zero-mean noise, with no projection
  and no extrinsic error, so calibration never fights a camera bias.
- **Calibration fits 9 parameters, not the whole error:** five joint offsets, one compliance shared
  by all joints, and base x, y and yaw. The link-origin errors are drawn and never fitted, so a
  residual is left by design.
- **`ctrl_servo` ignores `Calib.uni` on its first move**, where `ctrl_calibrated` uses it, so one
  `Calib` gives a different first approach in the two controllers.
- **`move(uni=True)` says "each joint finishes moving in the + direction".** The approach direction
  matters only for unloaded joints; the two loaded joints rest on the gravity side either way.
- **`so101.load_arm(jaw_angle=...)` is unused.** The moving jaw's mass is lumped at the tool point.

## Layout

| path | what |
|---|---|
| `cellrt/` | reference controller (runtime, policy, supervisor, release gate, recovery ladder), fault classes, campaign, scorecard |
| `cellsim/` | example cell model: SO-101 kinematics, lash, camera; `arms.py` the p50 and p90 profiles |
| `examples/toy_cell.py` | a device without the arm model |
| `third_party/so101/` | SO-101 description files, Apache-2.0 (see `NOTICE.md` there) |
| `tests/` | the test suite |
| `tools/` | `mutants.py` (breaks each check and confirms a test fails), `hang_probe.py` (runs the supervisor as a separate process against a frozen runtime) |
| `CLAIMS.md` | every claim in this README, with the test or command that shows it |

## Related tools

- [ros2_fault_injection](https://ros2-fault-injection.readthedocs.io/) injects faults into ROS 2
  topics, transforms and services and checks assertions during the experiment. This bench does not
  use ROS; it scores outcomes of whole cycles against the truth and a blind twin.
- [fault_injection_simulation](https://github.com/s1mhadri/fault_injection_simulation) injects
  faults into a Gazebo simulation of a Franka Panda and records rosbags. This bench counts silent
  failures and false stops per fault class rather than recording runs.
- [CamFITool](https://github.com/inomuh/camfitool) injects image faults into robot cameras; IM-FIT
  and ROSMutation mutate robot code. Both test the robustness of code; this bench measures what a
  cell's checks catch and stop.
- Simulators (Gazebo, Isaac Sim, MuJoCo) model the cell; this bench runs a cell model under faults
  and counts. Your simulator can be the device.

## About

Prepared by CyberNative AI LLC, an AI-run company, using AI agents. The code and this README were
written by AI agents. MIT licence (`LICENSE`). The files under `third_party/so101/` are not covered
by it: they keep their Apache License 2.0; see `third_party/so101/LICENSE` and
`third_party/so101/NOTICE.md`. Issues and pull requests are welcome.
