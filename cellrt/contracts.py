"""Wire contracts between policy, runtime and supervisor.

Every request rides in an authenticated, sequence-numbered envelope;
a request names the observation it rests on (`observed_at_ms`, `frame_id`) so staleness can be
judged by the runtime, not asserted by the policy; and the refusal list is longer.

Nothing here is a safety function. A software refusal is not an engineered limit.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import math
from dataclasses import asdict, dataclass

KINDS = ("grasp", "place", "release", "home")
STEP_OF = {k: i for i, k in enumerate(KINDS)}          # a cycle is exactly grasp, place, release, home
APPROACH = ("uni", "direct")

# Reasons a request is not executed. 
REASONS = ("joint_limit", "envelope", "speed", "torque", "stale_calibration", "stale_observation",
           "no_heartbeat", "stop_input", "supervisor_hold",
           "malformed", "unauthorized", "replay", "out_of_sequence", "unknown_policy", "not_a_nest",
           "error_exceeds_capture", "frozen_frame", "no_supervisor", "tracking_error", "log_unavailable")


@dataclass(frozen=True)
class MotionRequest:
    cycle: int
    step: int
    kind: str
    target_xyz_mm: tuple
    approach: str
    capture_radius_mm: float
    expected_error_mm_p99: float
    calibration_id: str
    policy_id: str
    policy_version: str
    deadline_ms: int            # oldest the observation may be when motion starts
    observed_at_ms: int         # camera time of the observation this request rests on
    frame_id: int               # camera frame counter of that observation


@dataclass(frozen=True)
class RuntimeReply:
    cycle: int
    step: int
    outcome: str                # done | refused | stopped | timed_out | duplicate
    reason: str = ""
    detail: str = ""
    duration_ms: int = 0
    torque_peak: float = 0.0


_FIELDS = {
    "cycle": int, "step": int, "kind": str, "target_xyz_mm": list, "approach": str,
    "capture_radius_mm": (int, float), "expected_error_mm_p99": (int, float), "calibration_id": str,
    "policy_id": str, "policy_version": str, "deadline_ms": int, "observed_at_ms": int, "frame_id": int,
}


class Malformed(ValueError):
    pass


def parse_request(obj) -> MotionRequest:
    """Strict: exact key set, exact types (bool is not a number), finite values, sane ranges.
    Property: no input makes this return a request with a non-finite or mistyped field; every
    other input raises Malformed."""
    if not isinstance(obj, dict):
        raise Malformed("not an object")
    if set(obj) != set(_FIELDS):
        raise Malformed(f"keys differ: missing {sorted(set(_FIELDS) - set(obj))}, extra {sorted(set(obj) - set(_FIELDS))}")
    for k, t in _FIELDS.items():
        v = obj[k]
        if isinstance(v, bool) or not isinstance(v, t):
            raise Malformed(f"{k}: wrong type {type(v).__name__}")
    xyz = obj["target_xyz_mm"]
    if len(xyz) != 3 or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in xyz):
        raise Malformed("target_xyz_mm: need three finite numbers")
    for k in ("capture_radius_mm", "expected_error_mm_p99"):
        if not math.isfinite(obj[k]) or obj[k] < 0 or obj[k] > 1000:
            raise Malformed(f"{k}: out of range")
    if obj["kind"] not in KINDS or obj["approach"] not in APPROACH:
        raise Malformed("kind or approach not in the list")
    if obj["step"] != STEP_OF[obj["kind"]]:
        raise Malformed("step does not match kind")
    if obj["cycle"] < 0 or not 0 <= obj["deadline_ms"] <= 3_600_000 or obj["observed_at_ms"] < 0 or obj["frame_id"] < 0:
        raise Malformed("counter or time out of range")
    for k in ("calibration_id", "policy_id", "policy_version"):
        if not 0 < len(obj[k]) <= 80:
            raise Malformed(f"{k}: empty or too long")
    return MotionRequest(**{**obj, "target_xyz_mm": tuple(float(x) for x in xyz),
                            "capture_radius_mm": float(obj["capture_radius_mm"]),
                            "expected_error_mm_p99": float(obj["expected_error_mm_p99"])})


def canonical(obj) -> str:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), allow_nan=False)


def seal(key: bytes, key_id: str, seq: int, body: dict) -> dict:
    """Envelope with HMAC-SHA256 over key id, sequence number and body. A shared key is the slice's
    stand-in for a per-device asymmetric key; see README, Limits."""
    text = canonical(body)
    mac = hmac.new(key, f"{key_id}|{seq}|{text}".encode(), hashlib.sha256).hexdigest()
    return {"key_id": key_id, "seq": seq, "body": text, "mac": mac}


def open_envelope(keys: dict, env) -> tuple:
    """-> (seq, body dict). Raises PermissionError('unauthorized') or Malformed.
    Authentication is checked before the body is parsed: unauthenticated bytes are never interpreted."""
    if not isinstance(env, dict) or set(env) != {"key_id", "seq", "body", "mac"}:
        raise Malformed("envelope shape")
    if not isinstance(env["key_id"], str) or isinstance(env["seq"], bool) or not isinstance(env["seq"], int) \
            or not isinstance(env["body"], str) or not isinstance(env["mac"], str):
        raise Malformed("envelope types")
    key = keys.get(env["key_id"])
    if key is None:
        raise PermissionError("unauthorized")
    want = hmac.new(key, f"{env['key_id']}|{env['seq']}|{env['body']}".encode(), hashlib.sha256).hexdigest()
    if not hmac.compare_digest(want, env["mac"]):
        raise PermissionError("unauthorized")
    try:
        body = json.loads(env["body"], parse_constant=lambda c: (_ for _ in ()).throw(Malformed(f"constant {c}")))
    except (json.JSONDecodeError, RecursionError) as e:
        raise Malformed(f"body is not JSON: {type(e).__name__}")
    return env["seq"], body


def request_dict(r: MotionRequest) -> dict:
    d = asdict(r)
    d["target_xyz_mm"] = list(r.target_xyz_mm)
    return d
