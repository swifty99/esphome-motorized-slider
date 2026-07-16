#!/usr/bin/env python3
"""Session capture tool for the motorslider bench (PicoScope 3203D via pico-bench).

MUST run under the pico-bench x86/Rosetta venv (py3.9):

    arch -x86_64 /Users/jan/dev/repos/pico-bench/.venv-x86/bin/python \
        analysis/scope-recipes/capture.py --preset alignment --out data/<date>/NN_scope.npz

Presets encode the probe plan from docs/protocols/probe-connections.md.
Probe 1 = Ch A, probe 2 = Ch B, EXT = TP-SYNC (GPIO14).
Arm the capture FIRST, then fire the lab experiment (EXT rising edge = program start).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, "/Users/jan/dev/repos/pico-bench/src")
from picobench.driver.ps3000a import Scope  # noqa: E402

# preset: (chA cfg, chB cfg, trigger, rate_hz, duration_s, pre_frac)
# channel cfg = (label, range_v, coupling) or None
# Session-1 dressing, verified by channel-ID capture 00_channel_id.npz:
# Ch A = wiper (ADC node), Ch B = AISEN shunt 0.2R, EXT = TP-SYNC GPIO14
# (EXT trigger did not fire pre-ground-fix; shunt-step trigger on B is primary).
# No PWM probe — PWM phase is recovered from the shunt current ripple in post.
PRESETS = {
    # channel identification / grounding sanity: both channels wide DC, free-run
    "diag": dict(
        a=("chA", 5.0, "DC"), b=("chB", 5.0, "DC"),
        trig=("NONE", 0, "rising"), rate=100e3, dur=5.0, pre=0.0),
    # Phase-1 gate closer: below-breakaway pwm_coupling puts a current step
    # through the shunt at an exact CSV timestamp -> cross-instrument alignment.
    "alignment": dict(
        a=("wiper", 5.0, "DC"), b=("isen_0r2", 0.2, "DC"),
        trig=("B", 0.025, "rising"), rate=100e3, dur=12.0, pre=0.05),
    # M1: wiper noise floor, single channel (rail comparison deferred).
    "noise": dict(
        a=("wiper_ac", 0.05, "AC"), b=None,
        trig=("NONE", 0, "rising"), rate=1e6, dur=10.0, pre=0.0),
    # M2: PWM coupling into the wiper; shunt ripple carries the PWM phase.
    "coupling": dict(
        a=("wiper_ac", 0.05, "AC"), b=("isen_0r2", 0.2, "DC"),
        trig=("B", 0.025, "rising"), rate=10e6, dur=1.0, pre=0.01),
    # M3: AISEN shunt current over the whole duty_ladder; wiper confirms stall.
    "ladder": dict(
        a=("wiper", 5.0, "DC"), b=("isen_0r2", 0.2, "DC"),
        trig=("B", 0.025, "rising"), rate=1e6, dur=35.0, pre=0.01),
}


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--preset", required=True, choices=sorted(PRESETS))
    p.add_argument("--out", required=True, help="output .npz path")
    p.add_argument("--rate", type=float, help="override sample rate [Hz]")
    p.add_argument("--dur", type=float, help="override duration [s]")
    p.add_argument("--trig", help="override trigger as SRC,VOLTS,DIR e.g. 'A,0.025,rising' "
                                  "(EXT path in the pico-bench driver is not yet HW-validated)")
    p.add_argument("--timeout", type=float, default=90.0,
                   help="max seconds to wait for trigger+capture")
    p.add_argument("--note", default="", help="free-text note stored in the npz")
    args = p.parse_args()

    cfg = dict(PRESETS[args.preset])
    if args.rate:
        cfg["rate"] = args.rate
    if args.dur:
        cfg["dur"] = args.dur
    if args.trig:
        src, volts, direction = args.trig.split(",")
        cfg["trig"] = (src.strip().upper(), float(volts), direction.strip())
    n_samples = int(cfg["rate"] * cfg["dur"])

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    with Scope() as scope:  # __enter__ opens the unit
        info = scope.info()
        print(f"scope: {info}")
        channels = []
        labels = {}
        for ch, key in (("A", "a"), ("B", "b")):
            if cfg[key] is not None:
                label, rng, coup = cfg[key]
                scope.set_channel(ch, rng, coup)
                channels.append(ch)
                labels[ch] = label
        trig_src, trig_v, trig_dir = cfg["trig"]
        if trig_src == "NONE":
            # free-running: auto-trigger immediately, capture starts on arm
            scope.set_simple_trigger("A", 0.0, "rising", enabled=False, auto_ms=1)
        else:
            scope.set_simple_trigger(trig_src, trig_v, trig_dir, auto_ms=0)

        print(f"ARMED [{args.preset}] {cfg['rate']:.0f} S/s x {cfg['dur']:.1f}s "
              f"({n_samples} samples/ch), trigger {trig_src} {trig_dir} @ {trig_v} V")
        print(">>> fire the lab experiment now <<<")
        t0 = time.time()
        cap = scope.capture_block(channels, cfg["rate"], n_samples,
                                  pre_frac=cfg["pre"], timeout_s=args.timeout)
        print(f"triggered+captured in {time.time() - t0:.1f}s, dt={cap.dt_s * 1e9:.0f} ns")

    data = {f"volts_{labels[ch]}": cap.volts[ch] for ch in channels}
    meta = dict(preset=args.preset, labels=labels, dt_s=cap.dt_s,
                rate_hz=cfg["rate"], duration_s=cfg["dur"],
                trigger=cfg["trig"], pre_frac=cfg["pre"],
                overflow={k: bool(v) for k, v in cap.overflow.items()},
                note=args.note, captured_at=time.strftime("%Y-%m-%dT%H:%M:%S"))
    np.savez_compressed(out, meta=json.dumps(meta), dt_s=cap.dt_s, **data)
    for ch in channels:
        v = cap.volts[ch]
        print(f"  ch{ch} [{labels[ch]}]: min={v.min():.4f} max={v.max():.4f} "
              f"mean={v.mean():.4f} V, overflow={cap.overflow[ch]}")
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
