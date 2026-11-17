"""Append-only, hash-chained event log. One JSON line per event; each line carries the hash of the
previous line, so a removed, reordered or edited line is found by `verify`.

What the log holds: every envelope received (as received), every reply, every sensor reading the
supervisor used, every supervisor decision, every recovery step, and the manifest in force.
What it never holds: camera images. A request carries numbers derived from an image, not the image
(README, Limits). Replay (`replay.py`) re-runs the runtime's decisions from the
logged inputs and must reproduce the logged replies exactly.
"""
from __future__ import annotations

import hashlib
import json
import os

GENESIS = "0" * 64


class EventLog:
    def __init__(self, path=None, capacity=None):
        self.path = path
        self.lines = []            # kept in memory too: the slice's runs are short
        self.prev = GENESIS
        self.capacity = capacity   # None = unlimited; a number simulates a full disk
        self.failed = False
        self._fh = open(path, "w") if path else None

    def append(self, t_ms: int, kind: str, **data) -> bool:
        """False when the event could not be stored. The runtime refuses motion while that is so."""
        if self.capacity is not None and len(self.lines) >= self.capacity:
            self.failed = True
            return False
        body = json.dumps({"t": t_ms, "k": kind, "d": data, "p": self.prev}, sort_keys=True, separators=(",", ":"))
        self.prev = hashlib.sha256(body.encode()).hexdigest()
        self.lines.append(body)
        if self._fh:
            self._fh.write(body + "\n")
            if kind in ("verdict", "recovery"):     # once a cycle: the events before a power cut are the ones a diagnosis wants
                self._fh.flush()
                os.fsync(self._fh.fileno())
        return True

    def anchor(self) -> dict:
        """The line count and head hash, to be sent off the machine (one line in the daily status
        message is enough). The chain alone shows accident: whoever holds the file can edit it and
        recompute every hash. Checked against an anchor held elsewhere, an edit before the anchor
        shows."""
        return {"n": len(self.lines), "head": self.prev}

    def close(self):
        if self._fh:
            self._fh.close()
            self._fh = None

    @property
    def nbytes(self):
        return sum(len(x) + 1 for x in self.lines)


def verify(lines, anchors=()) -> tuple:
    """-> (ok, index of the first bad line or -1, head hash). `anchors`: {"n", "head"} records that
    left the machine earlier; the log must still have the same head at each of those lengths."""
    want = {a["n"]: a["head"] for a in anchors}
    if want.get(0, GENESIS) != GENESIS:
        return False, 0, GENESIS
    prev = GENESIS
    for i, line in enumerate(lines):
        try:
            if json.loads(line)["p"] != prev:
                return False, i, prev
        except (ValueError, KeyError, TypeError):
            return False, i, prev
        prev = hashlib.sha256(line.encode()).hexdigest()
        if want.get(i + 1, prev) != prev:
            return False, i, prev                # the chain is consistent and is not the one that was anchored
    if any(n > len(lines) for n in want):
        return False, len(lines), prev           # lines that were anchored are gone
    return True, -1, prev


def events(lines, kind=None):
    for line in lines:
        e = json.loads(line)
        if kind is None or e["k"] == kind:
            yield e
