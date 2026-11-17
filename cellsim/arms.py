"""Two simulated arm profiles for the cell model (`cell.Mech.play_deg`, `scatter_deg`), built from
per-joint quantiles in a public table of SO-100/SO-101 recordings.

Source: holdcheck `holdcheck/corpus_summary.json` at commit be1ebd42, table `joints`
(https://github.com/CyberNative-AI/holdcheck/blob/be1ebd42/holdcheck/corpus_summary.json). The
corpus measured 1,574 public LeRobot datasets across several robot families. The `joints` table
uses only its SO-100 and SO-101 families: its unit is an arm aggregate, one contributor's datasets
of one family, 371 in all (226 SO-100, 145 SO-101); an aggregate is not one physical arm, and
there are not 371 recordings. Where the leader arm holds still, the follower's settled error split
by approach direction gives each joint's lost motion (`lost_motion_deg`; play is half of it, 308
to 348 aggregates per joint). Free holds give the free-hold settle scatter (`scatter_sd_free_deg`,
126 to 224 aggregates per joint): holds of about 1 s or more, where the recording has at least 8
in each approach direction. Where holdcheck can recover the gripper's encoder step, a free hold must
also have a small gripper error, a proxy for an empty gripper; where it cannot, it admits long holds
by duration alone (holdcheck.py L352 and L402). The published summary does not count how many holds
were admitted by duration alone. The data does not show that the arm touched nothing during them.

"p50" puts every joint at its own median over the aggregates, "p90" every joint at its own 90th
percentile. Neither is a real arm, nor the median or 90th-percentile arm: joints of one arm are not
equally loose. The encoder sits on the servo output shaft, so play in horns, screws and printed
links is not seen; these are lower bounds at the tool. They come from hobby arms' own recordings,
not from bench measurements of one arm.

Values in degrees, in so101.ARM_JOINTS order (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll).
"""
from __future__ import annotations

from dataclasses import replace

from .cell import Mech

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")

ARMS = {
    # the cell's own modelled arm: +/- 0.5 deg play, loaded joints on the gravity side, no settle scatter
    "model": None,
    "p50": dict(play_deg=(0.3735, 0.3955, 1.2964, 0.5713, 0.3625),
                scatter_deg=(0.1851, 0.3583, 0.3397, 0.1955, 0.1303)),
    "p90": dict(play_deg=(0.6114, 1.4502, 2.3291, 1.0547, 0.6152),
                scatter_deg=(0.3258, 0.7498, 0.7493, 0.6711, 0.3046)),
}


def mech_for(arm: str | None, base: Mech | None = None) -> Mech:
    """The Mech for a named arm ("model", "p50", "p90"; None means "model")."""
    base = base or Mech()
    p = ARMS[arm or "model"]
    return base if p is None else replace(base, **p)
