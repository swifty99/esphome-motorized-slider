"""Command one GLIDE move (continuous keep-rolling slow mode) and fetch the trace.

    cd analysis && uv run python servo-hil/glide_move.py <target%> [speed%/s] [watch_s] [dump.csv]

Enables "Servo Glide"=1 and applies the GLIDE knobs from the environment BEFORE the
move, so a slow Servo Speed uses GLIDE (never stop) instead of the stop-start CRAWL:
    GLIDE (1)  GLIDE_FLOOR (40)  GLIDE_STEP (0.4)  GLIDE_SETTLE (3)
    V_ROLL (30)  COAST_ZONE (0.8)
Set GLIDE=0 to A/B the same move as CRAWL. After the move it pulls the ring dump to
<dump.csv> (default /tmp/glide/last.csv) for crawl_smoothness.py. Host via $SERVO_HOST.
Ref: docs/glide-mode-spec.md.
"""
import asyncio
import os
import sys
import time
import urllib.request

from aioesphomeapi import APIClient

HOST = os.environ.get("SERVO_HOST", "192.168.178.115")

# env var -> HA number name -> default
KNOBS = [
    ("GLIDE", "Servo Glide", 1.0),
    ("GLIDE_FLOOR", "Servo Glide Floor", 40.0),
    ("GLIDE_STEP", "Servo Glide Step", 0.4),
    ("GLIDE_SETTLE", "Servo Glide Settle", 3.0),
    ("V_ROLL", "Servo Roll Floor", 30.0),
    ("COAST_ZONE", "Servo Coast Zone", 0.8),
]


async def main():
    target = float(sys.argv[1])
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0
    watch_s = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0
    dump_path = sys.argv[4] if len(sys.argv) > 4 else "/tmp/glide/last.csv"

    cli = APIClient(HOST, 6053, "")
    await cli.connect(login=True)
    ents, _ = await cli.list_entities_services()
    names = {e.key: e.name for e in ents if hasattr(e, "name")}
    key = {e.name: e.key for e in ents if hasattr(e, "name")}
    st = {}
    cli.subscribe_states(lambda s: st.__setitem__(
        names.get(getattr(s, "key", None)), getattr(s, "state", None)))
    await asyncio.sleep(1.5)

    applied = []
    for env, hname, dflt in KNOBS:
        v = float(os.environ.get(env, dflt))
        if hname in key:
            cli.number_command(key[hname], v)
            applied.append(f"{hname}={v:g}")
    await asyncio.sleep(0.3)
    print("knobs: " + "  ".join(applied))

    p0 = st.get("Servo Position")
    print(f"pre: pos={p0:.2f}%  state={st.get('Servo State')}")
    cli.number_command(key["Servo Speed"], speed)
    await asyncio.sleep(0.2)
    print(f">>> GLIDE Target={target}%  speed={speed} %/s")
    cli.number_command(key["Servo Target"], target)

    t0 = time.monotonic()
    last = None
    peak_v = 0.0
    while time.monotonic() - t0 < watch_s:
        await asyncio.sleep(0.1)
        s = st.get("Servo State")
        peak_v = max(peak_v, abs(st.get("Servo v_hat") or 0.0))
        if s != last:
            print(f"  [+{time.monotonic()-t0:5.2f}s] {str(s):8} pos={st.get('Servo Position'):.2f} "
                  f"p_hat={st.get('Servo p_hat'):.2f} v={st.get('Servo v_hat') or 0.0:+.0f} "
                  f"pulses={st.get('Servo Pulses')} th={st.get('Servo Thermal Avg') or 0.0:.3f}")
            last = s
        if s == "idle" and st.get("Servo Settled") and time.monotonic() - t0 > 0.5:
            break

    await asyncio.sleep(0.3)
    z = st.get("Servo Position")
    print(f"post: pos={z:.2f}%  settle={st.get('Servo Settle Flag')} "
          f"release={st.get('Servo Release Reason')} pulses={st.get('Servo Pulses')} "
          f"overshoots={st.get('Servo Overshoots')} peak|v|={peak_v:.0f} "
          f"thermal={st.get('Servo Thermal Avg') or 0.0:.3f}")
    print(f"      landing error = {z - target:+.2f}% (tol 0.30)")
    await cli.disconnect()

    os.makedirs(os.path.dirname(dump_path) or ".", exist_ok=True)
    try:
        urllib.request.urlretrieve(f"http://{HOST}/servo/dump.csv", dump_path)
        print(f"dump -> {dump_path}")
    except Exception as e:  # noqa: BLE001
        print(f"dump fetch failed: {e}")


asyncio.run(main())
