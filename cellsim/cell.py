"""Quasi-static bench-cell model: a low-cost arm, an overhead camera, fixtures with a capture radius.

What is modelled (all parameters explicit, defaults in README, Example cell parameters):
  - unit-to-unit variation: joint zero offsets, link-origin tolerances, base mounting error;
  - servo proportional-only position control: steady-state sag = gravity torque / kp;
  - gear lash: a joint whose gravity torque exceeds `preload_tau` rests against one side of its
    lash (repeatable); an unloaded joint lands anywhere inside +/- lash (not repeatable);
  - command resolution of the 12-bit encoder;
  - a camera that reports world positions with isotropic Gaussian noise.
What is not modelled: dynamics, vibration, contact forces, friction stick-slip during motion,
thermal drift, cable drag, vision failures. A capture radius stands in for the whole insertion
contact problem (see README, Example cell parameters).
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, replace

import numpy as np

from . import so101

DEG = np.pi / 180.0


@dataclass(frozen=True)
class Mech:
    kp: float = 17.8             # N·m/rad, TheRobotStudio joints_properties.xml; Menagerie uses 998.22
    lash_deg: float = 0.5        # +/- deg, the 'backlash' class in both upstream files
    preload_tau: float = 0.052   # N·m, upstream frictionloss, used as the lash take-up threshold
    enc_counts: int = 4096       # STS3215 12-bit encoder
    offset_sd_deg: float = 1.0   # per-unit joint zero offset sd (assumption, swept)
    link_sd_mm: float = 0.3      # per-unit link-origin tolerance sd, each axis (assumption)
    base_sd_mm: float = 2.0      # per-unit base mounting position sd in x and y (assumption)
    base_yaw_sd_deg: float = 0.5  # per-unit base mounting yaw sd (assumption)
    payload: float = 0.02        # kg at the tool point
    lash_model: str = "play"     # "play": hysteresis (link lags the motor inside the gap);
    #                              "random": an unloaded joint lands anywhere in the gap every move
    lash_jitter: float = 0.2     # play model: random part of the lash position, fraction of lash
    # Measured arm (optional). Per-joint play half-width and free-hold settle scatter sd, in deg, in
    # so101.ARM_JOINTS order, at lash_deg = MEASURED_BASE_LASH. When set, every joint is a play
    # operator of half-width play_deg[j] x lash_deg / MEASURED_BASE_LASH (so a lash fault still
    # scales the arm's own play) plus Gaussian settle scatter; the loaded/free split, lash_model and
    # lash_jitter are not used. See arms.py.
    play_deg: tuple | None = None
    scatter_deg: tuple | None = None


MEASURED_BASE_LASH = 0.5


def half_width_rad(mech: "Mech"):
    """Per-joint play half-width in rad (an array for a measured arm, a scalar otherwise)."""
    if mech.play_deg is None:
        return mech.lash_deg * DEG
    return np.asarray(mech.play_deg, float) * DEG * (mech.lash_deg / MEASURED_BASE_LASH)


@dataclass(frozen=True)
class Cam:
    sd_mm: float = 0.5           # per-axis position noise of one look (assumption, swept)


class Unit:
    """One physical arm in one cell: true kinematics plus hidden offsets."""

    def __init__(self, seed: int, mech: Mech, nominal: so101.Arm | None = None):
        self.seed = seed
        self.mech = mech
        self.nominal = nominal or so101.load_arm()
        rng = np.random.default_rng([seed, 7919])
        arm = copy.deepcopy(self.nominal)
        for l in arm.links:
            l.pos = l.pos + rng.normal(0, mech.link_sd_mm * 1e-3, 3)
        dxy = rng.normal(0, mech.base_sd_mm * 1e-3, 2)
        yaw = rng.normal(0, mech.base_yaw_sd_deg * DEG)
        arm.base_pos = arm.base_pos + np.array([dxy[0], dxy[1], 0.0])
        c, s = np.cos(yaw), np.sin(yaw)
        arm.base_rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]) @ arm.base_rot
        self.true = arm
        self.offsets = rng.normal(0, mech.offset_sd_deg * DEG, arm.n)
        self.res = 2 * np.pi / mech.enc_counts

    def with_mech(self, **kw):
        """Same hidden draws, different mechanism parameters (for sweeps)."""
        return Unit(self.seed, replace(self.mech, **kw), self.nominal)

    def _lash(self, u, tau, rng, y_prev):
        """Link angle for motor-side equilibrium u (N,5). Loaded joints rest on the gravity side of
        the gap. Unloaded joints follow the lash model."""
        m = self.mech
        if m.play_deg is not None:
            B = half_width_rad(m)
            y = u + B * rng.uniform(-1, 1, u.shape) if y_prev is None else np.clip(y_prev, u - B, u + B)
            return y + np.asarray(m.scatter_deg, float) * DEG * rng.standard_normal(u.shape)
        B = m.lash_deg * DEG
        loaded = np.abs(tau) > m.preload_tau
        y_loaded = u + np.sign(tau) * B * (1.0 - 0.1 * rng.random(u.shape))
        if m.lash_model == "random" or y_prev is None:
            y_free = u + B * rng.uniform(-1, 1, u.shape)
        elif m.lash_model == "play":
            y_free = np.clip(y_prev, u - B, u + B)
            y_free = np.clip(y_free + m.lash_jitter * B * rng.uniform(-1, 1, u.shape), u - B, u + B)
        else:
            raise ValueError(m.lash_model)
        return np.where(loaded, y_loaded, y_free)

    def settle(self, q_cmd, rng, y_prev=None):
        """Joint angles the arm actually reaches for commands q_cmd (N,5). `y_prev` is the link
        state before the move (None = unknown history, drawn uniformly inside the gap)."""
        m = self.mech
        q_c = np.round(np.atleast_2d(q_cmd) / self.res) * self.res
        q = q_c + self.offsets
        for _ in range(3):
            tau = self.true.gravity_torque(q, m.payload)
            q = q_c + self.offsets + tau / m.kp
        tau = self.true.gravity_torque(q, m.payload)
        return self._lash(q, tau, rng, y_prev)

    def reach(self, q_cmd, rng, y_prev=None):
        """After settling: true tool position (N,3), tilt from vertical (N,), link state (N,5)."""
        y = self.settle(q_cmd, rng, y_prev)
        p, a = self.true.fk(y)
        tilt = np.arccos(np.clip(-a[:, 2], -1, 1))
        return p, tilt, y

    def command_for(self, p_true, rng, approach=None):
        """The command a closed-loop human operator ends on when the tool is at p_true (used to
        generate demonstrations). `approach` (N,5) in {-1,+1} is the direction each joint was last
        moving; None = arbitrary (uniform inside the gap). In the play model the link lags the
        motor, so a joint that arrived moving + sits at the low side of the gap. Default arm only:
        with `play_deg` set (a p50 or p90 profile) it raises NotImplementedError."""
        m = self.mech
        if m.play_deg is not None:
            raise NotImplementedError(
                "command_for models the default arm only; a measured arm (play_deg set) is not supported")
        q, ok = self.true.ik_down(p_true)
        tau = self.true.gravity_torque(q, m.payload)
        B = m.lash_deg * DEG
        loaded = np.abs(tau) > m.preload_tau
        if approach is None or m.lash_model == "random":
            h_free = B * rng.uniform(-1, 1, q.shape)
        else:
            h_free = np.clip(-approach * B + m.lash_jitter * B * rng.uniform(-1, 1, q.shape), -B, B)
        h = np.where(loaded, np.sign(tau) * B * (1.0 - 0.1 * rng.random(q.shape)), h_free)
        q_c = q - self.offsets - tau / m.kp - h
        return np.round(q_c / self.res) * self.res, ok


def look(p, cam: Cam, rng):
    return p + rng.normal(0, cam.sd_mm * 1e-3, p.shape)


# ----------------------------------------------------------------------------------------------
# Work area and targets
# ----------------------------------------------------------------------------------------------
AREA = dict(x=(0.13, 0.25), y=(-0.10, 0.10))
Z_WORK = 0.03
HOME = np.array([[0.19, 0.0, 0.07]])


def sample_targets(n, rng, area=AREA, z=Z_WORK):
    x = rng.uniform(*area["x"], n)
    y = rng.uniform(*area["y"], n)
    return np.stack([x, y, np.full(n, z)], 1)


def home_state(unit: "Unit", n, rng):
    """Link state after power-up and a move to the home pose (lash history unknown)."""
    q, _ = unit.nominal.ik_down(HOME)
    return unit.settle(np.repeat(q, n, 0), rng, None)


def move(unit: "Unit", q, rng, y_prev, uni=False):
    """Move to command q. With `uni` the arm first goes to q minus three lash widths on every
    joint and then to q, so each joint finishes moving in the + direction (one extra short move).
    Returns p, tilt, y, moves."""
    if uni:
        m = unit.mech
        if m.play_deg is None:
            d = 3.0 * m.lash_deg * DEG
        else:  # three of the widest joint's play widths
            d = 3.0 * float(np.max(np.asarray(m.play_deg, float) * DEG)) * (m.lash_deg / MEASURED_BASE_LASH)
        _, _, y_prev = unit.reach(q - d, rng, y_prev)
        p, tilt, y = unit.reach(q, rng, y_prev)
        return p, tilt, y, 2
    p, tilt, y = unit.reach(q, rng, y_prev)
    return p, tilt, y, 1


# ----------------------------------------------------------------------------------------------
# Controllers. A placing function has the signature
#     place(target (N,3), rng, y_prev (N,5)) -> dict(p, tilt, valid, looks, moves, y)
# `looks` counts camera looks and `moves` counts arm moves, the two things that cost cycle time.
# ----------------------------------------------------------------------------------------------
def _out(p, tilt, valid, looks, moves, y):
    n = len(p)
    return dict(p=p, tilt=tilt, valid=valid, looks=np.broadcast_to(looks, (n,)).astype(float),
                moves=np.broadcast_to(moves, (n,)).astype(float), y=y)


@dataclass
class Calib:
    """Fitted per-unit correction: joint offsets, joint compliance 1/kp, base shift and yaw."""
    theta: np.ndarray            # (9,) = offsets(5), compliance(1), base dx, dy, dyaw
    rms_mm: float
    n_points: int
    uni: bool = False

    def model_arm(self, nominal):
        arm = copy.copy(nominal)
        dx, dy, yaw = self.theta[6:9]
        c, s = np.cos(yaw), np.sin(yaw)
        arm.base_pos = nominal.base_pos + np.array([dx, dy, 0.0])
        arm.base_rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]) @ nominal.base_rot
        return arm


def _calib_predict(nominal, theta, q_cmd, payload):
    arm = Calib(theta, 0.0, 0).model_arm(nominal)
    q = q_cmd + theta[:5]
    for _ in range(2):
        q = q_cmd + theta[:5] + theta[5] * arm.gravity_torque(q, payload)
    p, _ = arm.fk(q)
    return p


def calib_poses(k, rng):
    side = int(np.ceil(np.sqrt(k / 2)))
    xs = np.linspace(AREA["x"][0], AREA["x"][1], side)
    ys = np.linspace(AREA["y"][0], AREA["y"][1], side)
    pts = np.array([(x, y, z) for z in (Z_WORK, Z_WORK + 0.03) for x in xs for y in ys])
    return pts[rng.permutation(len(pts))[:k]]


def calibrate(unit: Unit, cam: Cam, rng, k=12, uni=False):
    """Automatic calibration, no human input: visit k poses spread over the work area at two
    heights (one after another, so lash history is whatever the path gives), look at the tool
    once per pose, fit 9 parameters by regularised Gauss-Newton."""
    tgt = calib_poses(k, rng)
    q_cmd, ok = unit.nominal.ik_down(tgt)
    q_cmd = q_cmd[ok]
    y = home_state(unit, 1, rng)
    p_true = []
    for i in range(len(q_cmd)):
        p, _, y, _ = move(unit, q_cmd[i:i + 1], rng, y, uni)
        p_true.append(p[0])
    p_obs = look(np.array(p_true), cam, rng)
    theta = np.zeros(9)
    # weak zero-mean prior keeps the wrist-roll offset (barely observable from a tool point) tame
    w_prior = 1e-3 / np.array([3 * DEG] * 5 + [0.2, 0.01, 0.01, 2 * DEG])
    payload = unit.mech.payload
    for _ in range(12):
        r0 = (_calib_predict(unit.nominal, theta, q_cmd, payload) - p_obs).ravel()
        J = np.zeros((r0.size, 9))
        for i in range(9):
            d = np.zeros(9)
            d[i] = 1e-6
            J[:, i] = ((_calib_predict(unit.nominal, theta + d, q_cmd, payload) - p_obs).ravel() - r0) / 1e-6
        A = np.vstack([J, np.diag(w_prior)])
        b = np.concatenate([-r0, -w_prior * theta])
        step, *_ = np.linalg.lstsq(A, b, rcond=None)
        theta = theta + step
        if np.max(np.abs(step)) < 1e-9:
            break
    r = _calib_predict(unit.nominal, theta, q_cmd, payload) - p_obs
    return Calib(theta, float(np.sqrt(np.mean(np.sum(r ** 2, 1))) * 1e3), int(len(q_cmd)), uni)


def calibrated_command(unit: Unit, cal: Calib, target_obs):
    """Command that the calibrated model predicts will put the tool at target_obs."""
    arm = cal.model_arm(unit.nominal)
    q_true, ok = arm.ik_down(target_obs)
    tau = arm.gravity_torque(q_true, unit.mech.payload)
    return q_true - cal.theta[:5] - cal.theta[5] * tau, ok


def ctrl_calibrated(unit: Unit, target, cam: Cam, rng, y_prev, cal: Calib):
    """Open loop on the per-unit calibrated model. Uses the approach style the calibration used."""
    obs = look(target, cam, rng)
    q, ok = calibrated_command(unit, cal, obs)
    p, tilt, y, mv = move(unit, q, rng, y_prev, cal.uni)
    return _out(p, tilt, ok, 1, mv, y)


def ctrl_servo(unit: Unit, target, cam: Cam, rng, y_prev, cal: Calib | None = None, max_iter=6,
               gain=0.8, stop_mm=None):
    """Look-and-move visual servo. First move from the calibrated (or nominal) model, then up to
    `max_iter` corrections, each from one fresh look at tool and target. An episode stops
    correcting once its observed lateral error is under `stop_mm` (default 2 camera sd)."""
    stop = (2.0 * cam.sd_mm if stop_mm is None else stop_mm) * 1e-3
    n = len(target)
    obs = look(target, cam, rng)
    if cal is None:
        q, ok = unit.nominal.ik_down(obs)
        model = unit.nominal
    else:
        q, ok = calibrated_command(unit, cal, obs)
        model = cal.model_arm(unit.nominal)
    p, tilt, y, mv = move(unit, q, rng, y_prev, False)
    looks = np.ones(n)
    moves = np.full(n, float(mv))
    active = np.ones(n, bool)
    for _ in range(max_iter):
        d = look(target, cam, rng) - look(p, cam, rng)
        looks += active
        active &= np.linalg.norm(d[:, :2], axis=1) >= stop
        if not active.any():
            break
        want = np.concatenate([gain * d, np.zeros((n, 2))], 1)
        J = model.jac5(q)
        dq = np.linalg.solve(J + 1e-9 * np.eye(5)[None], want[:, :, None])[:, :, 0]
        q = np.where(active[:, None], q + np.clip(dq, -0.2, 0.2), q)
        p_new, tilt_new, y_new = unit.reach(q, rng, y)
        p = np.where(active[:, None], p_new, p)
        tilt = np.where(active, tilt_new, tilt)
        y = np.where(active[:, None], y_new, y)
        moves += active
    return _out(p, tilt, ok, looks, moves, y)
