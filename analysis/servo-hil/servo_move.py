"""Command one servo move, watch it live, print a landing/settle summary.

    cd analysis && uv run python servo-hil/servo_move.py <target%> [speed%/s] [watch_s]

speed 0 = fast move; 1..45 = CRAWL. Fetch the trace separately with
    curl http://192.168.178.115/servo/dump.csv -o trace.csv
and characterize a crawl with crawl_analyze.py. Host via $SERVO_HOST.
"""
import asyncio, os, sys, time
from aioesphomeapi import APIClient

HOST = os.environ.get("SERVO_HOST", "192.168.178.115")


async def main():
    target = float(sys.argv[1])
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    watch_s = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0

    cli = APIClient(HOST, 6053, "")
    await cli.connect(login=True)
    ents, _ = await cli.list_entities_services()
    names = {e.key: e.name for e in ents if hasattr(e, "name")}
    key = {e.name: e.key for e in ents if hasattr(e, "name")}
    st = {}
    cli.subscribe_states(lambda s: st.__setitem__(
        names.get(getattr(s, "key", None)), getattr(s, "state", None)))
    await asyncio.sleep(1.5)

    p0 = st.get("Servo Position")
    print(f"pre: pos={p0:.2f}%  state={st.get('Servo State')}  thermal={st.get('Servo Thermal Avg')}")
    # always set Speed (0 = fast) so a prior CRAWL's speed doesn't leak in
    cli.number_command(key["Servo Speed"], speed)
    await asyncio.sleep(0.2)
    print(f">>> command Target = {target}%  (speed={speed})")
    cli.number_command(key["Servo Target"], target)

    t0 = time.monotonic()
    last_state = None
    peak_v = peak_thermal = 0.0
    while time.monotonic() - t0 < watch_s:
        await asyncio.sleep(0.1)
        s = st.get("Servo State")
        peak_v = max(peak_v, abs(st.get("Servo v_hat") or 0.0))
        peak_thermal = max(peak_thermal, st.get("Servo Thermal Avg") or 0.0)
        if s != last_state:
            print(f"  [+{time.monotonic()-t0:5.2f}s] state={s:8} pos={st.get('Servo Position'):.2f} "
                  f"p_hat={st.get('Servo p_hat'):.2f} v={st.get('Servo v_hat') or 0.0:+.1f} "
                  f"pulses={st.get('Servo Pulses')} th={st.get('Servo Thermal Avg') or 0.0:.3f}")
            last_state = s
        if s == "idle" and st.get("Servo Settled") and time.monotonic() - t0 > 0.5:
            break

    await asyncio.sleep(0.3)
    z = st.get("Servo Position")
    print(f"post: pos={z:.2f}%  p_hat={st.get('Servo p_hat'):.2f} target_echo={st.get('Servo Target Echo')} "
          f"settle={st.get('Servo Settle Flag')} release={st.get('Servo Release Reason')}")
    print(f"      pulses={st.get('Servo Pulses')} kicks={st.get('Servo Kicks')} "
          f"overshoots={st.get('Servo Overshoots')} peak|v|={peak_v:.1f} peak_thermal={peak_thermal:.3f} "
          f"missed={st.get('Servo Missed Ticks')} adc_err={st.get('Servo ADC Errors')} tick_p99={st.get('Servo Tick p99')}")
    print(f"      landing error = {z - target:+.2f}% (tol 0.30)")
    await cli.disconnect()


asyncio.run(main())
