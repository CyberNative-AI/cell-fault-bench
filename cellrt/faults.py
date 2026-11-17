"""The fault catalogue. Runtime-level faults act on the software; the four mechanical and camera faults
act on the physics (cellsim), so both families are scored on the
same cycles with the same supervisor.
"""
from __future__ import annotations

import json

import numpy as np

from .line import KID, Fault


# ---------------------------------------------------------------- wire
class DropPacket(Fault):
    name, step = "dropped_packet", 1

    def channel(self, line, envs):
        k = int(line.rng.integers(len(envs))) if self.step is None else self.step
        return [e for i, e in enumerate(envs) if i != k]


class DupPacket(Fault):
    name = "duplicated_packet"

    def channel(self, line, envs):
        k = int(line.rng.integers(len(envs)))
        return envs[: k + 1] + [envs[k]] + envs[k + 1:]


class Reorder(Fault):
    name = "reordered_packets"          # release arrives before place

    def channel(self, line, envs):
        e = list(envs)
        e[1], e[2] = e[2], e[1]
        return e


class CorruptPacket(Fault):
    name = "corrupted_packet"           # one bit flips on the wire: the MAC no longer matches

    def channel(self, line, envs):
        e = [dict(x) for x in envs]
        b = e[1]["body"]
        e[1]["body"] = b[:10] + ("1" if b[10] != "1" else "2") + b[11:]
        return e


# ---------------------------------------------------------------- commands nobody should obey
class InjectUnauthorized(Fault):
    name = "unauthorized_command"       # someone on the wire without the key sends "release" mid-carry

    def channel(self, line, envs):
        body = json.loads(envs[2]["body"])
        fake = {"key_id": KID, "seq": envs[0]["seq"] + 1000, "body": json.dumps(body), "mac": "0" * 64}
        return envs[:1] + [fake] + envs[1:]


class InjectReplay(Fault):
    name = "replayed_command"           # a recorded, correctly signed "release" from an earlier cycle

    def before_cycle(self, line):
        super().before_cycle(line)
        self.hit = self.hit and getattr(line, "_old_release", None) is not None

    def channel(self, line, envs):
        out = envs[:1] + [line._old_release] + envs[1:]
        return out


class RecordRelease(Fault):
    """Helper: remembers each cycle's release envelope so InjectReplay has something real to replay."""
    name, p = "_record", 1.0

    def channel(self, line, envs):
        if len(envs) >= 3:
            line._old_release, line._cur_release = getattr(line, "_cur_release", None), envs[2]
        return envs


class Malformed(Fault):
    name, variant = "malformed_command", "nan"

    def requests(self, line, reqs):
        r = [dict(x) for x in reqs]
        v = self.variant if self.variant != "mix" else ("nan", "missing", "type", "extra", "huge")[int(line.rng.integers(5))]
        b = r[1]
        if v == "nan":
            b["target_xyz_mm"] = [float("nan"), b["target_xyz_mm"][1], b["target_xyz_mm"][2]]
        elif v == "missing":
            del b["deadline_ms"]
        elif v == "type":
            b["capture_radius_mm"] = "3.0"
        elif v == "extra":
            b["joint_targets"] = [0, 0, 0, 0, 0]
        elif v == "huge":
            b["target_xyz_mm"] = [1e308 * 10, 0.0, 0.0]
        r[1] = json.dumps(b)                          # a raw body: json.dumps writes NaN and Infinity tokens
        return r


# ---------------------------------------------------------------- model errors
class ModelGross(Fault):
    name = "model_error_gross"          # a target far outside the cell

    def requests(self, line, reqs):
        r = [dict(x) for x in reqs]
        r[1]["target_xyz_mm"] = [r[1]["target_xyz_mm"][0] + 400.0, r[1]["target_xyz_mm"][1], 30.0]
        return r


class ModelOffset(Fault):
    name, mm = "model_error_offset", 4.0   # a plausible target inside the cell, off by `mm`

    def requests(self, line, reqs):
        r = [dict(x) for x in reqs]
        a = line.rng.uniform(0, 2 * np.pi)
        x = list(r[1]["target_xyz_mm"])
        x[0] += self.mm * np.cos(a)
        x[1] += self.mm * np.sin(a)
        r[1]["target_xyz_mm"] = x
        return r


class UnitMixup(Fault):
    name = "model_error_units"          # metres sent where millimetres are expected

    def requests(self, line, reqs):
        r = [dict(x) for x in reqs]
        for b in r[:2]:
            b["target_xyz_mm"] = [v * 0.001 for v in b["target_xyz_mm"]]
        return r


class StaleFrame(Fault):
    """The look behind the request is old and the fixtured scene has not changed since: the part
    arrived, the look was taken, and the cell then sat idle for `age_ms` (a slow camera pipeline, a
    wait on the line downstream). Time really passes; nothing is rewritten."""
    name, age_ms = "stale_observation", 30_000

    def before_cycle(self, line):
        super().before_cycle(line)
        if self.hit:
            line.idle_ms = self.age_ms


class EarlyLook(Fault):
    """The look is recent by the clock but was taken before the part arrived: a buffered frame of the
    nest as it was `early_ms` ago. In a fixtured cell every part lands in the same place, so the model
    cannot show harm from it; with a camera-derived pick target it is a look at the wrong part."""
    name, early_ms = "look_before_part_arrived", 3_000

    def before_cycle(self, line):
        super().before_cycle(line)
        if self.hit:
            line.look_early_ms = self.early_ms


# ---------------------------------------------------------------- persistent: processes and links
class PolicyDead(Fault):
    name, persistent, clears = "lost_heartbeat_policy_dead", True, ("restart_policy",)

    def start(self, line):
        self.first = True

    def requests(self, line, reqs):
        if self.first:                  # dies after the grasp of the cycle in progress
            self.first = False
            line.policy.alive = False
            return reqs[:1]
        return reqs

    def stop(self, line):
        line.policy.alive = True


class StaleCalibration(Fault):
    """The policy still holds the calibration from before the last recalibration. Blind, its targets
    are off by the difference between the two fits (drawn at onset, about 1.5 mm)."""
    name, persistent, clears = "wrong_calibration_id", True, ("resync", "restart_policy")

    def start(self, line):
        a = line.rng.uniform(0, 2 * np.pi)
        self.d = (1.5 * np.cos(a), 1.5 * np.sin(a))
        line.policy.calibration_id = "cal-0"
        line.policy.bias_mm = self.d

    def stop(self, line):
        line.policy.bias_mm = (0.0, 0.0)


class FrozenCamera(Fault):
    name, persistent, clears, hands = "frozen_frame", True, ("restart_policy",), None

    def start(self, line):
        line.policy.frozen_frame = (line.now, line.frame)
        line.policy.deadline_ms = 10 ** 6       # worst case: the policy also claims the frame may be any age

    def stop(self, line):
        line.policy.frozen_frame, line.policy.deadline_ms = None, 10_000


class RuntimeHung(Fault):
    name, persistent, clears = "runtime_hang", True, ("restart_runtime",)

    def start(self, line):
        line.rt.hung = True


class SupervisorDead(Fault):
    name, persistent, clears = "supervisor_dead", True, ("restart_supervisor",)

    def start(self, line):
        line.sup.alive = False
        line.now += 2000                 # time passes with no heartbeat


class LogFull(Fault):
    name, persistent, clears = "log_storage_full", True, ("rotate_log",)

    def start(self, line):
        line.log.capacity = len(line.log.lines)


class BadUpdate(Fault):
    """A new policy version is activated. `mm` = how far its place targets are off. 4 mm trips the
    cell-map check; 2 mm passes it and fails at the fixture some of the time."""
    name, persistent, clears, mm = "bad_update", True, ("rollback",), 4.0

    def start(self, line):
        self.name = f"bad_update_{self.mm:g}mm"
        a = line.rng.uniform(0, 2 * np.pi)
        line.update("1.1.0", bias_mm=(self.mm * np.cos(a), self.mm * np.sin(a)))


# ---------------------------------------------------------------- persistent: devices (hands needed)
class SeatStuck(Fault):
    name, persistent, clears, hands, how = "seat_sensor", True, ("swap_seat_sensor",), "swap_seat_sensor", "high"
    needs = "seat_stuck"           # the device field (or model) this fault acts on

    def start(self, line):
        self.name = f"seat_sensor_{self.how}"
        line.cell.faults.seat_stuck = self.how

    def stop(self, line):
        line.cell.faults.seat_stuck = None


class SeatGate(Fault):
    """One of the four corner gates is wrong; the other three work."""
    name, persistent, clears, hands = "seat_gate", True, ("swap_seat_sensor",), "swap_seat_sensor"
    needs = "seat_corner"           # the device field (or model) this fault acts on
    corner, how = 2, "high"

    def start(self, line):
        self.name = f"seat_gate_{self.how}"
        line.cell.faults.seat_corner = {self.corner: self.how}

    def stop(self, line):
        line.cell.faults.seat_corner = {}


class GripperMissing(Fault):
    name, persistent, clears, hands = "gripper_sensor_missing", True, ("swap_gripper",), "swap_gripper"
    needs = "gripper_stuck"           # the device field (or model) this fault acts on

    def start(self, line):
        line.cell.faults.gripper_stuck = "missing"

    def stop(self, line):
        line.cell.faults.gripper_stuck = None


class Stall(Fault):
    name, persistent, clears, hands = "motor_stall", True, ("free_arm",), "free_arm"
    needs = "stall"           # the device field (or model) this fault acts on

    def start(self, line):
        line.cell.faults.stall = True

    def stop(self, line):
        line.cell.faults.stall = False


class TorqueSpike(Fault):
    name, persistent, clears, hands = "torque_over_limit", True, ("free_arm",), "free_arm"
    needs = "torque_spike"           # the device field (or model) this fault acts on

    def start(self, line):
        line.cell.faults.torque_spike = True

    def stop(self, line):
        line.cell.faults.torque_spike = False


class BlockedNest(Fault):
    name, persistent, clears, hands = "part_across_nest", True, ("clear_nest",), "clear_nest"
    needs = "blocked"           # the device field (or model) this fault acts on

    def start(self, line):
        line.cell.blocked = True


# ---------------------------------------------------------------- persistent: four mechanical and camera faults (physics)
class Physical(Fault):
    persistent = True
    needs = "physics"           # the device field (or model) this fault acts on
    fault, value = "base_shift_mm", 1.0

    def start(self, line):
        self.name = f"{self.fault}={self.value:g}"
        self._old = dict(lash_deg=line.phys.unit.mech.lash_deg, cam_sd_mm=line.phys.cam_sd).get(self.fault)
        line.phys.apply_fault(self.fault, self.value)
        if self.fault in ("base_shift_mm", "offset_drift_deg"):
            self.clears, self.hands = ("recalibrate",), None
        elif self.fault == "lash_deg":
            self.clears, self.hands = ("swap_joint_module",), "swap_joint_module"
        else:
            self.clears, self.hands = ("clean_camera",), "clean_camera"

    def stop(self, line):
        if self.fault in ("lash_deg", "cam_sd_mm"):       # the swapped module or cleaned lens restores the old value
            line.phys.apply_fault(self.fault, self._old)
            line.recalibrate()
