# Third-party data: SO-101 robot description

- `so101.xml`: MuJoCo Menagerie `robotstudio_so101/so101.xml`, fetched 2026-11-15 04:05Z from
  `https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/robotstudio_so101/so101.xml`
  (sha256 `982c02c027ec0c62d3459d0a6eb00dfb7dc13a451992e72ce99addc71f209805`). Apache License 2.0 (`LICENSE`).
  Derived by Menagerie from TheRobotStudio/SO-ARM100 `Simulation/SO101/so101_new_calib.xml`.
- `joints_properties.upstream.xml`: TheRobotStudio/SO-ARM100 `Simulation/SO101/joints_properties.xml`,
  fetched at the same time (sha256 `abbcc92761943befe3a24b1c13e82c62c5b8c2914303818f3c870b1f51cd45ed`). Apache License 2.0
  per the upstream repository.

Unmodified copies. `cellsim/so101.py` reads only body frames, joint axes and ranges, inertial masses and
centres, and the `gripperframe` site from `so101.xml`. Meshes are not used and not copied.
The two files disagree on the servo position gain (17.8 vs 998.22 N·m/rad); `cellsim/cell.py` `Mech.kp`
uses 17.8 from `joints_properties.upstream.xml`. Neither is a measured servo stiffness.
