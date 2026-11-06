"""Characterize the CRAWL pulse train (the source of the clacking) from a servo
dump CSV. A "clack" = one drive pulse (u_applied != 0 burst).

    curl http://192.168.178.115/servo/dump.csv -o crawl.csv
    python servo-hil/crawl_analyze.py crawl.csv

Reports clacks/s, per-pulse width + displacement + duty — the acoustic-relevant
cadence. Compare crawl_impulse_duty / crawl_max_width_ms settings.
"""
import sys


def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or line.startswith("t_ms"):
                continue
            p = line.strip().split(",")
            if len(p) < 13:
                continue
            rows.append(dict(t=int(p[0]), z=float(p[1]), u=float(p[5]), state=int(p[9])))
    return rows


def main():
    rows = load(sys.argv[1])
    crawl_idx = [i for i, r in enumerate(rows) if r["state"] == 5]  # CRAWL=5
    if not crawl_idx:
        print("no CRAWL state in this trace")
        return
    span = rows[crawl_idx[0]:crawl_idx[-1] + 1]
    dur_s = (span[-1]["t"] - span[0]["t"]) / 1000.0

    pulses, i = [], 0
    while i < len(span):
        if abs(span[i]["u"]) > 0.01:
            j = i
            while j < len(span) and abs(span[j]["u"]) > 0.01:
                j += 1
            z0 = span[max(0, i - 3)]["z"]
            z1 = span[min(len(span) - 1, j + 60)]["z"]
            pulses.append(dict(w=span[j - 1]["t"] - span[i]["t"] + 1, duty=span[i]["u"], dz=z1 - z0))
            i = j
        else:
            i += 1

    n = len(pulses)
    z0, z1 = span[0]["z"], span[-1]["z"]
    print(f"CRAWL span: {dur_s:.2f} s, {z0:.1f}% -> {z1:.1f}% ({z1 - z0:+.1f}%), "
          f"achieved rate ~{abs(z1 - z0) / dur_s:.1f} %/s")
    print(f"clacks (drive pulses): {n}  =>  {n / dur_s:.1f} clacks/s")
    if n:
        ws = sorted(p["w"] for p in pulses)
        dzs = sorted(abs(p["dz"]) for p in pulses)
        print(f"  pulse width  ms: min {ws[0]} med {ws[n // 2]} max {ws[-1]}")
        print(f"  per-clack move %: min {dzs[0]:.2f} med {dzs[n // 2]:.2f} max {dzs[-1]:.2f}")
        print(f"  duty per pulse:  {pulses[0]['duty']:+.2f}")


if __name__ == "__main__":
    main()
