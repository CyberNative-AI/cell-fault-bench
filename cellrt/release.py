"""Versions, signed release bundles, two slots with an atomic switch, a shadow-replay gate, rollback.

A manifest names everything that decides behaviour: policy, calibration, runtime, configuration.
The runtime refuses any request that names a different policy or calibration (contracts.py), so a
half-applied update cannot move the arm.

Update path:  verify signature and payload hash -> refuse a release number that is not newer
(downgrade) -> write the inactive slot -> SHADOW REPLAY: feed the unit's own recent logged cycles
to the new policy and count what the runtime's checks would refuse and how far its targets moved
-> switch the pointer (one atomic rename) -> CANARY: the first cycles run under a tighter rule and
any breach switches back.

Boot falls back to a slot that verifies. Tested, simulated in files: a torn write of the staged
slot, an active slot that fails its hash, a missing or malformed pointer, and a rollback cut off
half way (tests/test_cellrt.py). Not tested: a real power cut, or storage that reorders writes. The
pointer is written with a file fsync and a rename, with no fsync of the directory, and boot does
not rewrite a lost pointer.

HMAC with a shared key stands in for a vendor signature here; a product signs with an offline
private key and devices hold only the public key (README, Limits).
"""
from __future__ import annotations

import hashlib
import hmac
import json
import os
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class Manifest:
    release: int
    policy_id: str
    policy_version: str
    calibration_id: str
    runtime_version: str
    config_sha256: str

    def public(self) -> dict:
        return asdict(self)

    def with_calibration(self, calibration_id: str) -> "Manifest":
        return Manifest(**{**asdict(self), "calibration_id": calibration_id})


class BadBundle(ValueError):
    pass


def make_bundle(key: bytes, manifest: Manifest, payload: dict) -> bytes:
    body = json.dumps({"manifest": manifest.public(), "payload": payload}, sort_keys=True, separators=(",", ":"))
    mac = hmac.new(key, body.encode(), hashlib.sha256).hexdigest()
    return json.dumps({"body": body, "mac": mac}).encode()


def open_bundle(key: bytes, raw: bytes) -> tuple:
    """-> (Manifest, payload). Raises BadBundle for anything that is not exactly what was signed."""
    try:
        outer = json.loads(raw)
        body, mac = outer["body"], outer["mac"]
        if not isinstance(body, str) or not isinstance(mac, str):
            raise TypeError
    except (ValueError, KeyError, TypeError):
        raise BadBundle("not a bundle")
    if not hmac.compare_digest(hmac.new(key, body.encode(), hashlib.sha256).hexdigest(), mac):
        raise BadBundle("signature")
    inner = json.loads(body)
    try:
        return Manifest(**inner["manifest"]), inner["payload"]
    except (TypeError, KeyError):
        raise BadBundle("manifest shape")


class Slots:
    """Slots `a` and `b` and a pointer file.

    A bundle has one of three names: `slot_X.staged` (installed, never run), `slot_X.bundle` (activated
    at some time) and `slot_X.rejected` (rolled back from). Only a `.bundle` can boot. So when the
    pointer file is lost or unreadable, which on a consumer flash card after a power cut is an ordinary
    event, the device cannot come up on a release that was never activated or on one its canary
    rejected. `activate` and `rollback` each rename the bundle first
    and move the pointer second, each step one os.replace."""

    def __init__(self, root: str, key: bytes):
        self.root, self.key = root, key
        os.makedirs(root, exist_ok=True)

    def _p(self, name):
        return os.path.join(self.root, name)

    def _write_atomic(self, name, data: bytes):
        tmp = self._p(name + ".tmp")
        with open(tmp, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self._p(name))

    def pointer(self) -> dict:
        """Any content that is not exactly the expected shape counts as no pointer."""
        try:
            with open(self._p("pointer.json")) as f:
                p = json.load(f)
        except (OSError, ValueError):
            p = None
        if (not isinstance(p, dict) or set(p) != {"active", "previous"} or p["active"] not in ("a", "b")
                or p["previous"] not in ("a", "b", None)):
            return {"active": None, "previous": None}
        return p

    def read(self, slot, state="bundle"):
        try:
            with open(self._p(f"slot_{slot}.{state}"), "rb") as f:
                return open_bundle(self.key, f.read())
        except (OSError, BadBundle, ValueError):
            return None

    def boot(self) -> tuple:
        """-> (slot, Manifest, payload) of the slot to run: the active one, or the other if the active
        slot does not verify (torn write, bit rot). With no readable pointer: the newest release that
        was ever activated and never rejected. Raises BadBundle if nothing verifies."""
        ptr = self.pointer()
        got = {s: self.read(s) for s in ("a", "b")}
        if ptr["active"]:
            order = [ptr["active"]] + [s for s in ("a", "b") if s != ptr["active"]]
        else:
            order = sorted((s for s in got if got[s]), key=lambda s: -got[s][0].release)
        for s in order:
            if got[s]:
                return s, got[s][0], got[s][1]
        raise BadBundle("no slot verifies")

    def factory(self, raw: bytes):
        open_bundle(self.key, raw)
        self._write_atomic("slot_a.bundle", raw)
        self._write_atomic("pointer.json", json.dumps({"active": "a", "previous": None}).encode())

    def install(self, raw: bytes, crash_after: str | None = None) -> str:
        """Verify and stage in the inactive slot. Does not switch and cannot boot. `crash_after`
        simulates power loss: 'partial' leaves half a file under the staged name (the worst a
        non-atomic writer could do)."""
        man, _ = open_bundle(self.key, raw)
        cur_slot, cur, _ = self.boot()
        floor = max([cur.release] + [r[0].release for r in (self.read(s, "rejected") for s in ("a", "b")) if r])
        if man.release <= floor:
            raise BadBundle(f"release {man.release} is not newer than {floor} (running or rejected)")
        target = "b" if cur_slot == "a" else "a"
        if crash_after == "partial":
            with open(self._p(f"slot_{target}.staged"), "wb") as f:
                f.write(raw[: len(raw) // 2])
            raise KeyboardInterrupt("power lost during slot write")
        self._write_atomic(f"slot_{target}.staged", raw)
        return target

    def activate(self, slot: str):
        if not self.read(slot, "staged"):
            raise BadBundle("slot does not verify")
        ptr = self.pointer()
        os.replace(self._p(f"slot_{slot}.staged"), self._p(f"slot_{slot}.bundle"))
        self._write_atomic("pointer.json", json.dumps({"active": slot, "previous": ptr["active"] or self._other(slot)}).encode())

    @staticmethod
    def _other(slot):
        return "a" if slot == "b" else "b"

    def rollback(self) -> str:
        """Back to the other slot. The release left behind is renamed first, so from that instant it
        cannot boot, pointer or no pointer."""
        cur = self.boot()[0]
        prev = self._other(cur)
        if not self.read(prev):
            raise BadBundle("previous slot does not verify")
        os.replace(self._p(f"slot_{cur}.bundle"), self._p(f"slot_{cur}.rejected"))
        self._write_atomic("pointer.json", json.dumps({"active": prev, "previous": None}).encode())
        return prev


def shadow_replay(policy_fn, logged_cycles: list, admit, max_refused_share=0.0, max_shift_mm=1.0) -> dict:
    """Run a candidate policy over observations this unit logged, without moving anything.

    logged_cycles: [{"obs": {...}, "targets": {"grasp": [x, y, z], "place": [x, y, z]}}] taken from the event log.
    policy_fn(obs) -> list of request dicts.  admit(request dict) -> "" or a refusal reason
    (the runtime's own checks, called without a cell).
    Passes only if no more than `max_refused_share` of cycles would be refused and no target moved
    more than `max_shift_mm` from what the unit did when those cycles were good.
    """
    refused, worst, reasons = 0, 0.0, {}
    for c in logged_cycles:
        bad = ""
        for r in policy_fn(c["obs"]):
            why = admit(r)
            if why:
                bad = why
                break
            old = c["targets"].get(r.get("kind"))
            if old:
                x = r["target_xyz_mm"]
                worst = max(worst, ((x[0] - old[0]) ** 2 + (x[1] - old[1]) ** 2) ** 0.5)
        if bad:
            refused += 1
            reasons[bad] = reasons.get(bad, 0) + 1
    n = max(1, len(logged_cycles))
    ok = refused / n <= max_refused_share and worst <= max_shift_mm
    return dict(passed=ok, cycles=len(logged_cycles), refused=refused, reasons=reasons, worst_target_shift_mm=round(worst, 3))
