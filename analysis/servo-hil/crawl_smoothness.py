#!/usr/bin/env python3
"""crawl_smoothness.py -- objective roughness / noise scorecard for CRAWL spans.

Slow ("CRAWL") motion of the belt slider is synthesized as a train of full-torque
micro-pulses: each pulse breaks the slider away from its friction cliff, it briefly
flies, the pulse is cut, and it re-sticks. That stop/re-stick is the audible "clack".
This tool quantifies how rough / loud a crawl is so tuning candidates can be A/B'd
purely on numbers.

Usage:
    cd analysis && uv run python servo-hil/crawl_smoothness.py <dump.csv> [more.csv ...]
    python3 servo-hil/crawl_smoothness.py <dump.csv> [more.csv ...]

Stdlib only (no numpy / pandas). Percentiles, std and CV are computed by hand.

Dump columns (0-indexed):
    0 t_ms 1 z 2 p_hat 3 v_hat 4 u_cmd 5 u_applied 6 v_des 7 ff 8 tick_us
    9 state 10 settle 11 release 12 clamped
    state:   0 IDLE 1 MOVE 2 BRAKE 3 PULSE 4 WAIT 5 CRAWL 6 RELEASED
    settle:  0 none 1 reached 2 gave_up 3 adopted
    release: 0 none 1 grab 2 interlock 3 watchdog 4 fault 5 command

Definitions
-----------
CRAWL span : rows from the first to the last state==CRAWL(5) row, inclusive. The
             drive pulses fired from CRAWL are recorded as state==PULSE(3) rows that
             live *inside* that span, so scanning the span for |u_applied|>0.01 finds
             every clack.
pulse/clack: one contiguous run of rows with |u_applied| > 0.01.
coast      : the window of up to COAST_MS (60 ms) after a pulse ends (capped at the
             next pulse's start), during which the slider flies and re-sticks. The
             re-stick -- a sharp drop in |v_hat| -- is the acoustic impact.

Per-pulse metrics (median and 90th pct across all pulses):
    width_ms      : pulse duration.
    dz %          : |p_hat travel| over pulse+coast (observer position).
    peak |v_hat|  : max |v_hat| during pulse+coast (%/s) -- how hard it flew.
    peak decel    : max positive per-ms drop in |v_hat| during pulse+coast
                    ((%/s)/ms) -- the clack-impact / jerk proxy.

Cadence / continuity:
    clacks/s      : pulses / span seconds.
    gap ms        : idle time between end of one pulse and start of the next;
                    reported as median and CV (std/mean) = "cadence regularity"
                    (lower CV = more regular = smoother-sounding).
    kept-rolling  : fraction of span ticks with |v_hat| > 10 %/s (higher = more
                    continuous motion, less stop-start).
    achieved rate : Δp_hat / span seconds, compared to requested rate if the trace
                    carries one (v_des during crawl; the pulse-train crawl leaves
                    v_des==0, so it is usually reported n/a).

Health:
    Flags FALSE-RELEASE? if the crawl episode contains any RELEASED(6) state,
    settle==adopted(3) or release!=none. These bench runs are unattended (no hand),
    so any grab/drag/watchdog release is a false positive. Reports the p_hat at
    which it released.

Roughness score (composite)
---------------------------
    roughness = clacks_per_sec * median(peak_decel_per_ms) / 100

One number, monotonic in "more / louder impacts = worse": it multiplies how *often*
the slider clacks (clacks/s) by how *hard* a typical re-stick hits (median per-ms
velocity collapse, the jerk proxy), scaled by /100 into a readable range. A dead
crawl (no motion, no clacks) scores ~0; a fast, frequently-clacking crawl scores
high. Use it only to rank real crawls -- pair it with the HEALTH flag, since a
false-released crawl can score deceptively low simply because it never moved.
"""
import sys

STATE = {0: "IDLE", 1: "MOVE", 2: "BRAKE", 3: "PULSE", 4: "WAIT", 5: "CRAWL",
         6: "RELEASED", 7: "GLIDE"}
SETTLE = {0: "none", 1: "reached", 2: "gave_up", 3: "adopted"}
RELEASE = {0: "none", 1: "grab", 2: "interlock", 3: "watchdog", 4: "fault", 5: "command"}

CRAWL = 5
GLIDE = 7
SLOW_STATES = (CRAWL, GLIDE)   # both slow modes are pulse trains -> same scorecard
IDLE = 0
COAST_MS = 60          # coast window after each pulse (ms)
ROLL_THRESH = 10.0     # %/s, "kept rolling" velocity threshold
U_EPS = 0.01           # |u_applied| above this = driving


def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#") or s.startswith("t_ms"):
                continue
            p = s.split(",")
            if len(p) < 13:
                continue
            try:
                rows.append(dict(
                    t=int(p[0]), z=float(p[1]), p_hat=float(p[2]), v_hat=float(p[3]),
                    u_cmd=float(p[4]), u=float(p[5]), v_des=float(p[6]), ff=float(p[7]),
                    tick_us=int(p[8]), state=int(p[9]), settle=int(p[10]),
                    release=int(p[11]), clamped=int(p[12])))
            except (ValueError, IndexError):
                continue
    return rows


def percentile(vals, q):
    """Linear-interpolation percentile, q in [0,100]. vals need not be sorted."""
    if not vals:
        return 0.0
    xs = sorted(vals)
    if len(xs) == 1:
        return xs[0]
    idx = (q / 100.0) * (len(xs) - 1)
    lo = int(idx)
    hi = min(lo + 1, len(xs) - 1)
    frac = idx - lo
    return xs[lo] * (1 - frac) + xs[hi] * frac


def median(vals):
    return percentile(vals, 50)


def mean(vals):
    return sum(vals) / len(vals) if vals else 0.0


def pstdev(vals):
    if len(vals) < 2:
        return 0.0
    m = mean(vals)
    return (sum((x - m) ** 2 for x in vals) / len(vals)) ** 0.5


def find_pulses(span):
    """Return list of (start_idx, end_idx) index pairs into span for each contiguous
    |u_applied|>U_EPS run. end_idx is inclusive (last driving row)."""
    pulses = []
    i, n = 0, len(span)
    while i < n:
        if abs(span[i]["u"]) > U_EPS:
            j = i
            while j < n and abs(span[j]["u"]) > U_EPS:
                j += 1
            pulses.append((i, j - 1))
            i = j
        else:
            i += 1
    return pulses


def analyze(rows):
    """Return a dict of metrics for one trace, or {'empty': reason} if no usable CRAWL."""
    crawl_idx = [i for i, r in enumerate(rows) if r["state"] in SLOW_STATES]
    if not crawl_idx:
        return {"empty": "no CRAWL(5)/GLIDE(7) rows in trace"}
    # The recorder is a free-running ring that can hold SEVERAL moves. Split the
    # slow-state rows into episodes (a gap > EPISODE_GAP_MS starts a new move) and
    # score only the LAST (most recent) one — otherwise the span stretches across
    # moves + the idle between them and the scorecard is meaningless.
    EPISODE_GAP_MS = 500
    episodes = [[crawl_idx[0]]]
    for i in crawl_idx[1:]:
        if rows[i]["t"] - rows[episodes[-1][-1]]["t"] > EPISODE_GAP_MS:
            episodes.append([i])
        else:
            episodes[-1].append(i)
    ep = episodes[-1]
    s0, s1 = ep[0], ep[-1]
    span = rows[s0:s1 + 1]
    if len(span) < 2:
        return {"empty": "CRAWL span too short (<2 rows)"}

    span_s = (span[-1]["t"] - span[0]["t"]) / 1000.0
    if span_s <= 0:
        return {"empty": "CRAWL span has zero duration"}

    pulses = find_pulses(span)

    widths, dzs, peak_vs, peak_decels, gaps = [], [], [], [], []
    for k, (a, b) in enumerate(pulses):
        end_t = span[b]["t"]
        widths.append(end_t - span[a]["t"] + 1)

        # coast end: up to COAST_MS after pulse end, but not past the next pulse start
        coast_limit = end_t + COAST_MS
        next_start_t = span[pulses[k + 1][0]]["t"] if k + 1 < len(pulses) else None
        if next_start_t is not None:
            coast_limit = min(coast_limit, next_start_t)
        c = b
        while c + 1 < len(span) and span[c + 1]["t"] <= coast_limit:
            c += 1  # c = last row inside pulse+coast window

        base = span[a - 1] if a > 0 else span[a]     # position just before the pulse
        dzs.append(abs(span[c]["p_hat"] - base["p_hat"]))

        # peak |v_hat| and peak per-ms drop across pulse+coast
        pv, pd = 0.0, 0.0
        for m in range(a, c + 1):
            av = abs(span[m]["v_hat"])
            if av > pv:
                pv = av
            if m + 1 <= c:
                dt = span[m + 1]["t"] - span[m]["t"]
                if dt <= 0:
                    dt = 1
                drop = av - abs(span[m + 1]["v_hat"])
                if drop > 0:
                    d = drop / dt
                    if d > pd:
                        pd = d
        peak_vs.append(pv)
        peak_decels.append(pd)

        # inter-pulse gap: from this pulse end to next pulse start
        if k + 1 < len(pulses):
            gaps.append(span[pulses[k + 1][0]]["t"] - end_t)

    n = len(pulses)
    clacks_per_s = n / span_s

    rolling = sum(1 for r in span if abs(r["v_hat"]) > ROLL_THRESH)
    kept_rolling = rolling / len(span)

    dp = span[-1]["p_hat"] - span[0]["p_hat"]
    achieved_rate = dp / span_s
    direction = "FORWARD" if dp > 0 else ("REVERSE" if dp < 0 else "STALLED")

    req_v = max((abs(r["v_des"]) for r in span), default=0.0)
    requested_rate = req_v if req_v > 0.05 else None

    med_decel = median(peak_decels) if peak_decels else 0.0
    roughness = clacks_per_s * med_decel / 100.0

    # ---- health: scan the full crawl episode (span + trailing non-IDLE rows so a
    # terminal RELEASED run right after the last CRAWL row is caught) ----
    j = s1 + 1
    while j < len(rows) and rows[j]["state"] != IDLE:
        j += 1
    episode = rows[s0:j]
    released = None
    for r in episode:
        if r["state"] == 6 or r["settle"] == 3 or r["release"] != 0:
            released = r
            break

    return {
        "span_s": span_s,
        "p0": span[0]["p_hat"], "p1": span[-1]["p_hat"], "dp": dp,
        "direction": direction,
        "achieved_rate": achieved_rate, "requested_rate": requested_rate,
        "n_pulses": n, "clacks_per_s": clacks_per_s,
        "widths": widths, "dzs": dzs, "peak_vs": peak_vs, "peak_decels": peak_decels,
        "gaps": gaps, "kept_rolling": kept_rolling,
        "roughness": roughness, "released": released,
    }


def fmt_block(path, m):
    name = path.rsplit("/", 1)[-1]
    W = 80
    out = ["=" * W, " " + name, "-" * W]
    if "empty" in m:
        out.append("  no scorecard: " + m["empty"])
        out.append("=" * W)
        return "\n".join(out)

    req = "n/a" if m["requested_rate"] is None else "%.1f %%/s" % m["requested_rate"]
    out.append(" CRAWL span     : %.2f s   %.2f%% -> %.2f%%  (d %+.2f%%)   dir %s"
               % (m["span_s"], m["p0"], m["p1"], m["dp"], m["direction"]))
    out.append(" achieved rate  : %+.1f %%/s      requested(v_des): %s"
               % (m["achieved_rate"], req))
    out.append(" pulses (clacks): %d   =>  %.1f clacks/s" % (m["n_pulses"], m["clacks_per_s"]))
    out.append(" kept-rolling   : %.1f %%   (span ticks with |v_hat| > %.0f %%/s)"
               % (100.0 * m["kept_rolling"], ROLL_THRESH))
    out.append("")
    out.append(" per-pulse                 median        p90")
    rows_ = [
        ("width_ms",            m["widths"],      "%8.1f"),
        ("travel dz %",         m["dzs"],         "%8.2f"),
        ("peak |v_hat| %/s",    m["peak_vs"],     "%8.1f"),
        ("peak decel (%/s)/ms", m["peak_decels"], "%8.2f"),
    ]
    for label, vals, f in rows_:
        med = median(vals) if vals else 0.0
        p90 = percentile(vals, 90) if vals else 0.0
        out.append(("   %-22s" + f + "  " + f) % (label, med, p90))
    out.append("")
    gm = median(m["gaps"]) if m["gaps"] else 0.0
    gmean = mean(m["gaps"]) if m["gaps"] else 0.0
    cv = (pstdev(m["gaps"]) / gmean) if gmean > 0 else 0.0
    out.append(" inter-pulse gap: median %6.1f ms   CV %.2f  (cadence regularity, lower=smoother)"
               % (gm, cv))
    out.append(" roughness score: %6.2f   (clacks/s x median peak-decel / 100)" % m["roughness"])
    out.append("")
    r = m["released"]
    if r is None:
        out.append(" HEALTH         : OK  (no RELEASED / adopted / release event)")
    else:
        out.append(" HEALTH         : ***FALSE-RELEASE?***  state=%s settle=%s release=%s @ p_hat=%.2f%%"
                   % (STATE.get(r["state"], r["state"]),
                      SETTLE.get(r["settle"], r["settle"]),
                      RELEASE.get(r["release"], r["release"]),
                      r["p_hat"]))
    out.append("=" * W)
    return "\n".join(out)


def summary_line(path, m):
    name = path.rsplit("/", 1)[-1]
    if "empty" in m:
        return "  %-16s  (no CRAWL: %s)" % (name, m["empty"])
    flag = "FALSE-RELEASE?" if m["released"] is not None else "ok"
    return ("  %-16s  %5.1f clacks/s  rough %6.2f  keep-roll %4.1f%%  %s"
            % (name, m["clacks_per_s"], m["roughness"], 100.0 * m["kept_rolling"], flag))


def main(argv):
    paths = argv[1:]
    if not paths:
        print("usage: crawl_smoothness.py <dump.csv> [more.csv ...]")
        return 2
    results = []
    for path in paths:
        try:
            rows = load(path)
        except OSError as e:
            m = {"empty": "cannot read file: %s" % e}
        else:
            m = {"empty": "no data rows"} if not rows else analyze(rows)
        results.append((path, m))
        print(fmt_block(path, m))
        print()
    print("SUMMARY (higher roughness = louder/rougher crawl; keep-roll = more continuous):")
    for path, m in results:
        print(summary_line(path, m))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
