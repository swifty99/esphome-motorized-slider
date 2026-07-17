"""One-shot slider_servo status over the ESPHome API. Retries through a reboot.

    cd analysis && uv run python servo-hil/servo_status.py

Prints the Servo* entities (state, position, observer, A9 tick health, reset
reason) + a few dormant-lab entities for the uptime/reboot check. Host via
$SERVO_HOST (default 192.168.178.115).
"""
import asyncio, os, sys, time
from aioesphomeapi import APIClient

HOST = os.environ.get("SERVO_HOST", "192.168.178.115")


async def main(collect_s=4.0, connect_timeout=75.0):
    deadline = time.monotonic() + connect_timeout
    cli = APIClient(HOST, 6053, "")
    while True:
        try:
            await cli.connect(login=True)
            break
        except Exception as e:  # noqa: BLE001 — retry through a reboot
            if time.monotonic() > deadline:
                print(f"CONNECT FAILED after {connect_timeout:.0f}s: {e}")
                return 1
            await asyncio.sleep(2.0)
    info = await cli.device_info()
    ents, _ = await cli.list_entities_services()
    names = {e.key: e.name for e in ents if hasattr(e, "name")}
    states = {}
    cli.subscribe_states(lambda s: states.__setitem__(
        names.get(getattr(s, "key", None)), getattr(s, "state", None)))
    await asyncio.sleep(collect_s)
    await cli.disconnect()

    print(f"esphome {getattr(info, 'esphome_version', '?')} @ {HOST}")
    servo = {k: v for k, v in states.items() if k and k.startswith("Servo")}
    other = {k: v for k, v in states.items()
             if k in ("Lab Uptime", "Lab ADC Errors", "Slider Position", "Lab Player State")}
    print("--- SERVO ---")
    for k in sorted(servo):
        print(f"  {k:24} {servo[k]}")
    print("--- other (lab dormant) ---")
    for k in sorted(other):
        print(f"  {k:24} {other[k]}")
    return 0


sys.exit(asyncio.run(main()))
