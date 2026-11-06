"""A1 acceptance sweep — the tuning loop's objective metric.

    cd analysis && uv run python servo-hil/a1_sweep.py [N=40] [seed=1]

N randomized moves (seed => reproducible sequence, so a retune is compared on
the SAME moves). Live telemetry only, thermal-guarded. Prints per-move lines and
a per-direction scorecard: gave_up, overshoot-rate, |landing error|, pulses,
out-of-tol, settle time. Host via $SERVO_HOST.

Baseline to beat (firmware commit e28f965, 2026-07-17, seed 1):
  gave_up 0 | overshoot fwd 0% rev 11% | max|err| 0.43% | out-of-tol 12% | max pulses 3
"""
import asyncio, os, sys, time, random, statistics as stats
from aioesphomeapi import APIClient

HOST = os.environ.get("SERVO_HOST", "192.168.178.115")
LO, HI = 15.0, 85.0      # target range (avoid the extreme ends)
MIN_STEP = 5.0           # every move is a real move
TOL = 0.30
THERMAL_PAUSE = 0.35     # cool down if the thermal EMA climbs past this


async def main():
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    rng = random.Random(seed)

    cli = APIClient(HOST, 6053, "")
    await cli.connect(login=True)
    ents, _ = await cli.list_entities_services()
    names = {e.key: e.name for e in ents if hasattr(e, "name")}
    key = {e.name: e.key for e in ents if hasattr(e, "name")}
    st = {}
    cli.subscribe_states(lambda s: st.__setitem__(
        names.get(getattr(s, "key", None)), getattr(s, "state", None)))
    await asyncio.sleep(1.5)
    cli.number_command(key["Servo Speed"], 0.0)  # fast moves
    await asyncio.sleep(0.2)

    cur = float(st.get("Servo Position", 50.0))
    results = []
    for i in range(N):
        while True:
            tgt = round(rng.uniform(LO, HI) * 2) / 2
            if abs(tgt - cur) >= MIN_STEP:
                break
        direction = "fwd" if tgt > cur else "rev"
        if float(st.get("Servo Thermal Avg", 0.0) or 0.0) > THERMAL_PAUSE:
            print("  [thermal high, cooling 15 s]")
            await asyncio.sleep(15.0)

        t0 = time.monotonic()
        cli.number_command(key["Servo Target"], tgt)
        left = settled = False
        while time.monotonic() - t0 < 7.0:
            await asyncio.sleep(0.05)
            s = st.get("Servo State")
            if s and s != "idle":
                left = True
            if left and s == "idle" and st.get("Servo Settled"):
                settled = True
                break
        move_t = time.monotonic() - t0
        await asyncio.sleep(0.65)  # let the 500 ms-interval sensors publish finals
        z = float(st.get("Servo Position"))
        rec = dict(i=i, frm=round(cur, 2), tgt=tgt, dir=direction, err=round(z - tgt, 3),
                   pulses=int(st.get("Servo Pulses") or 0), over=int(st.get("Servo Overshoots") or 0),
                   kicks=int(st.get("Servo Kicks") or 0), settle=st.get("Servo Settle Flag"),
                   rel=st.get("Servo Release Reason"), t=round(move_t, 2), ok=settled)
        results.append(rec)
        flag = "" if (settled and abs(rec["err"]) <= TOL and rec["over"] == 0) else "  <--"
        print(f"  {i:2} {rec['frm']:5.1f}->{tgt:5.1f} {direction} err{rec['err']:+.2f} "
              f"pul{rec['pulses']} ov{rec['over']} k{rec['kicks']} {rec['settle']:7} {move_t:4.1f}s{flag}")
        cur = z
        await asyncio.sleep(0.8)

    await cli.disconnect()

    def agg(rs, label):
        if not rs:
            return
        errs = [abs(r["err"]) for r in rs]
        ovr = sum(1 for r in rs if r["over"] > 0) / len(rs)
        oot = sum(1 for r in rs if abs(r["err"]) > TOL) / len(rs)
        print(f"{label:5} n={len(rs):2}  |err| med {stats.median(errs):.3f} max {max(errs):.3f}  "
              f"pulses med {stats.median([r['pulses'] for r in rs]):.0f} max {max(r['pulses'] for r in rs)}  "
              f"overshoot {ovr:.0%} (sum {sum(r['over'] for r in rs)})  out-of-tol {oot:.0%}  "
              f"settle med {stats.median([r['t'] for r in rs]):.1f}s")

    print("\n=== A1 SCORECARD ===")
    agg(results, "all")
    agg([r for r in results if r["dir"] == "fwd"], "fwd")
    agg([r for r in results if r["dir"] == "rev"], "rev")
    bad = [r for r in results if not r["ok"] or r["settle"] == "gave_up" or r["rel"] != "none"]
    print(f"gave_up/failures/release: {len(bad)}  moves {[r['i'] for r in bad]}")


asyncio.run(main())
