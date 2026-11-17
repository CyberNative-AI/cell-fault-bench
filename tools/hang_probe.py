"""Wall-clock probe: a supervisor in its own process watches a runtime process that freezes.

    python3 tools/hang_probe.py --trials 40 > results/hang_probe.json

The runtime process writes a heartbeat byte on a pipe every 20 ms and holds an "enable" flag it
cannot keep alive by itself. The supervisor process drops the flag when no heartbeat arrives for
`--timeout-ms`. Each trial freezes the runtime at a random moment (SIGSTOP: the process is alive
and holds its pipes, which is the case a crash detector misses) and measures, on the parent's
clock, the time from the freeze to the flag dropping. Also counts false trips over a quiet period.

This measures the mechanism on this Linux host under whatever load it has. It is not a measurement
of a motor stopping: on a machine the flag is a relay or a drive's safe-torque-off input, and the
time that matters includes the drive and the mechanics.
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import random
import signal
import statistics as st
import time


def runtime_proc(w):
    while True:
        os.write(w, b".")
        time.sleep(0.02)


def supervisor_proc(r, enable, timeout_s):
    import select
    last = time.monotonic()
    while True:
        ready, _, _ = select.select([r], [], [], 0.005)
        now = time.monotonic()
        if ready:
            os.read(r, 4096)
            last = now
            if not enable.value:
                enable.value = 1          # re-arm after the runtime comes back (a product would require a reset)
        elif now - last > timeout_s and enable.value:
            enable.value = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=40)
    ap.add_argument("--timeout-ms", type=int, default=150)
    ap.add_argument("--quiet-s", type=float, default=20.0)
    a = ap.parse_args()
    ctx = mp.get_context("fork")
    r, w = os.pipe()
    enable = ctx.Value("i", 1)
    rt = ctx.Process(target=runtime_proc, args=(w,), daemon=True)
    sup = ctx.Process(target=supervisor_proc, args=(r, enable, a.timeout_ms / 1000), daemon=True)
    rt.start()
    sup.start()
    time.sleep(0.5)
    # quiet period: any drop here is a false trip
    false_trips, t_end = 0, time.monotonic() + a.quiet_s
    was = 1
    while time.monotonic() < t_end:
        v = enable.value
        false_trips += (was == 1 and v == 0)
        was = v
        time.sleep(0.001)
    rnd = random.Random(1)
    lat, missed = [], 0
    for _ in range(a.trials):
        time.sleep(0.2 + rnd.random() * 0.3)
        t0 = time.monotonic()
        os.kill(rt.pid, signal.SIGSTOP)
        while enable.value and time.monotonic() - t0 < 2.0:
            time.sleep(0.0005)
        if enable.value:
            missed += 1
        else:
            lat.append((time.monotonic() - t0) * 1000)
        os.kill(rt.pid, signal.SIGCONT)
        t1 = time.monotonic()
        while not enable.value and time.monotonic() - t1 < 2.0:
            time.sleep(0.001)
    rt.kill()
    sup.kill()
    lat.sort()
    print(json.dumps(dict(
        kind="MEASURED on this host: process mechanism only, not a motor stop",
        heartbeat_ms=20, timeout_ms=a.timeout_ms, trials=a.trials, missed=missed,
        freeze_to_enable_drop_ms=dict(min=round(lat[0], 1), median=round(st.median(lat), 1),
                                      p95=round(lat[int(0.95 * (len(lat) - 1))], 1), max=round(lat[-1], 1)) if lat else None,
        quiet_period_s=a.quiet_s, false_trips_in_quiet_period=false_trips,
        loadavg=os.getloadavg(), cpus=os.cpu_count()), indent=1))


if __name__ == "__main__":
    main()
