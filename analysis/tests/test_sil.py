"""Phase-5 SIL tripwire suite (controller-spec.md §3, T1-T17).

Every tripwire is asserted on sim_runner CSV traces, and every one is proven
NON-VACUOUS by a negative control: the same checker must FAIL against the
matching seeded bug (--bug N, compiled only under SLIDER_SIL_BUGS).

Gate (plan.md Phase 5): all tripwires green incl. negative controls.
"""

from __future__ import annotations

import csv
import subprocess
from pathlib import Path

import numpy as np
import pytest

SIL_DIR = Path(__file__).resolve().parents[2] / "sil"
RUNNER = SIL_DIR / "build" / "sim_runner"
RUNNER_PROD = SIL_DIR / "build" / "sim_runner_prod"

V_MIN_CMD = 120.0
U_MAX = 0.9
TOL = 0.30

# legal (from -> to) state pairs, spec §2 rows 1-29 (self-loops implicit)
LEGAL = {
    ("IDLE", "MOVE"), ("IDLE", "WAIT"), ("IDLE", "CRAWL"), ("IDLE", "RELEASED"),
    ("MOVE", "BRAKE"), ("MOVE", "RELEASED"),
    ("BRAKE", "WAIT"), ("BRAKE", "RELEASED"),
    ("PULSE", "WAIT"), ("PULSE", "CRAWL"), ("PULSE", "BRAKE"),
    ("PULSE", "RELEASED"),
    ("WAIT", "IDLE"), ("WAIT", "MOVE"), ("WAIT", "CRAWL"), ("WAIT", "PULSE"),
    ("WAIT", "BRAKE"), ("WAIT", "RELEASED"),
    ("CRAWL", "PULSE"), ("CRAWL", "WAIT"), ("CRAWL", "BRAKE"),
    ("CRAWL", "RELEASED"),
    ("RELEASED", "IDLE"), ("RELEASED", "MOVE"), ("RELEASED", "WAIT"),
    ("RELEASED", "CRAWL"),
}


# ------------------------------------------------------------ infrastructure

@pytest.fixture(scope="session", autouse=True)
def build_runner():
    if not (SIL_DIR / "build").exists():
        subprocess.run(["cmake", "-B", "build", "-DCMAKE_BUILD_TYPE=Release"],
                       cwd=SIL_DIR, check=True, capture_output=True)
    subprocess.run(["cmake", "--build", "build"], cwd=SIL_DIR, check=True,
                   capture_output=True)


class Run:
    def __init__(self, summary: dict, trace: dict):
        self.s = summary
        self.t = trace

    def rows(self, mask=None):
        n = len(self.t["tick"])
        return np.arange(n) if mask is None else np.flatnonzero(mask)


def run_sim(tmp_path: Path, scenario: str, bug: int = 0, recorder: int | None = None,
            binary: Path = RUNNER, tag: str = "s") -> Run:
    sf = tmp_path / f"{tag}.txt"
    sf.write_text(scenario, encoding="utf-8")
    out = tmp_path / f"{tag}_{bug}_{recorder}.csv"
    cmd = [str(binary), str(sf), "--out", str(out), "--bug", str(bug)]
    if recorder is not None:
        cmd += ["--recorder", str(recorder)]
    proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
    summary = {}
    for line in proc.stdout.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            try:
                summary[k] = float(v) if "." in v else int(v)
            except ValueError:
                summary[k] = v
    with out.open() as f:
        rows = list(csv.DictReader(f))
    trace: dict = {}
    for k in rows[0]:
        col = [r[k] for r in rows]
        if k in ("state", "flag", "reason"):
            trace[k] = np.array(col)
        else:
            trace[k] = np.array([float(x) for x in col])
    return Run(summary, trace)


# ------------------------------------------------------------------ scenarios

SC_MOVE = "p0 30\nat 0.05 target 70\n"
SC_MOVE_REV = "p0 80\nat 0.05 target 30\n"
SC_SHORT_REV = "p0 70\nat 0.05 target 68\n"
SC_BAND = "p0 90\nperturb 0.07 0.07 0.1 -0.1\nat 0.05 target 65\n"
SC_GRAB = "p0 20\nduration 6\nrest 1.5\nat 0.05 target 80\nat 0.2 grab 1.5\n"
SC_GRAB_LONG = "p0 20\nduration 8\nrest 0\nat 0.05 target 80\nat 0.2 grab 9.0\n"
SC_REST_HAND = ("p0 40\nduration 5\nrest 0\nat 0.05 target 55\n"
                "at 2.5 step 5\n")
SC_CRAWL = "p0 20\nduration 6\nat 0.05 target 60 speed 20\n"
SC_CRAWL_GRAB = ("p0 20\nduration 6\nrest 1.5\nat 0.05 target 60 speed 20\n"
                 "at 1.0 grab 2.0\n")
SC_CRAWL_DRAG = ("p0 20\nduration 6\nrest 1.5\nat 0.05 target 60 speed 15\n"
                 "at 1.0 drag 55 0.5\n")
SC_FREEZE = "p0 30\nduration 5\nrest 0\nat 0.05 target 70\nat 0.15 freeze on\n"
SC_ABORT = "p0 20\nduration 4\nrest 0\nat 0.05 target 80\nat 0.3 abort on\n"
SC_RETARGET2 = ("p0 20\nduration 6\nat 0.05 target 80\nat 0.25 target 40\n"
                "at 0.3 target 60\n")


# ------------------------------------------------------------------ checkers
# Each returns (ok, msg). Positive test asserts ok; the negative control
# asserts NOT ok under the seeded bug — the proof the checker can fail.

def chk_t1_silence(r: Run):
    at_rest = (r.t["state"] == "IDLE") | (r.t["state"] == "RELEASED")
    bad = at_rest & (r.t["u_cmd"] != 0.0)
    return not bad.any(), f"{bad.sum()} nonzero commands at rest"


def chk_t2_no_creep(r: Run):
    v = np.abs(r.t["v_des"])
    active = v > 0.0
    if ((v[active] < V_MIN_CMD) & (v[active] > 0)).any():
        return False, "v_des below the cliff floor"
    move = (r.t["state"] == "MOVE") & (np.abs(r.t["u_cmd"]) > 0.01)
    u = np.abs(r.t["u_cmd"][move])
    plateau = (u > 0.05) & (u < 0.45)
    return not plateau.any(), f"{plateau.sum()} MOVE duties in the dead band"


def chk_t3_ff_sign(r: Run):
    m = (r.t["state"] == "MOVE") & (r.t["v_des"] != 0) & (r.t["ff"] != 0)
    ok = np.sign(r.t["ff"][m]) == np.sign(r.t["v_des"][m])
    return ok.all(), f"{(~ok).sum()} FF ticks keyed off desired direction"


def chk_t4_grab_stall(r: Run, grab_start: int = 200):
    gd = r.s["grab_detect_tick"]
    ok = gd >= 0 and (gd - grab_start) <= 600
    return ok, f"grab detected at {gd} (grab at {grab_start})"


def chk_t5_offset_adopted(r: Run, step_tick: int = 2500):
    win = slice(step_tick + 200, step_tick + 300)
    err = np.abs(r.t["p_hat"][win] - r.t["pos"][win])
    return float(err.max()) < 1.0, f"p_hat error {err.max():.2f} after offset"


def chk_t6_release_torque_free(r: Run, grab_end: int = 1700):
    gd = int(r.s["grab_detect_tick"])
    if gd < 0:
        return False, "no grab declared"
    after = r.t["u_cmd"][gd + 2:grab_end]
    return (after == 0.0).all(), "torque after grab declaration"


def chk_t7_fwd_landing(r: Run):
    latch = int(r.s["settle_latch_tick"])
    if latch <= 0 or r.s["settle_flag"] != "reached":
        return False, f"no reached-latch ({r.s['settle_flag']})"
    pre = r.t["u"][:latch]
    nz = np.flatnonzero(pre)
    return len(nz) > 0 and pre[nz[-1]] > 0, "last motion-causing duty not forward"


def chk_t8_honest_flag(r: Run):
    flag = r.s["settle_flag"]
    if r.s["final_state"] != "IDLE":
        return False, "did not latch"
    err = abs(r.s["hold"] - r.s["target"])
    if flag == "reached":
        return err <= 0.8, f"claims reached with {err:.2f}% error"
    return flag in ("gave_up", "adopted"), f"flag {flag}"


def chk_t9_terminal(r: Run):
    ok = r.s["final_state"] in ("IDLE", "RELEASED")
    return ok, f"final state {r.s['final_state']}"


def chk_t10_pulse_domain(r: Run):
    # PULSE segments: the entry tick (decision) is legitimately zero-duty;
    # every EMISSION tick must be exactly +-0.8 and the emission count must
    # sit inside the identified table domain [2, 50] ms
    pulse = r.t["state"] == "PULSE"
    idx = np.flatnonzero(pulse)
    if len(idx) == 0:
        return True, "no pulses"
    seg_starts = np.concatenate([[0], np.flatnonzero(np.diff(idx) > 1) + 1])
    seg_bounds = np.append(seg_starts, len(idx))
    widths = []
    for a, b in zip(seg_bounds[:-1], seg_bounds[1:]):
        u = r.t["u_cmd"][idx[a:b]]
        nz = u[u != 0.0]
        if len(nz) and not np.allclose(np.abs(nz), 0.8, atol=1e-6):
            return False, "pulse duty != 0.8"
        widths.append(len(nz))
    widths = np.array(widths)
    # a segment may be cut short (retarget row 17); only overlong is illegal
    return bool((widths <= 50).all() and (widths >= 1).all()), \
        f"pulse widths {sorted(set(widths.tolist()))}"


def chk_t11_duty_bounded(r: Run):
    over = np.abs(r.t["u_cmd"]) > U_MAX + 1e-6
    return not over.any(), f"{over.sum()} ticks above u_max"


def chk_t12_sensor_wins(r: Run, freeze_tick: int = 150):
    win = slice(freeze_tick + 100, None)
    dev = np.abs(r.t["p_hat"][win] - r.t["z"][win])
    return float(dev.max()) < 4.0, f"p_hat deviated {dev.max():.1f}% from z"


def chk_t14_same_tick_clamp(r: Run, abort_tick: int = 300):
    i = abort_tick
    if np.abs(r.t["u_cmd"][i]) < 0.05:
        return False, "controller not active at the abort tick (vacuous)"
    ok = r.t["clamped"][i] == 1 and r.t["u"][i] == 0.0
    if not ok:
        return False, f"clamp late: u={r.t['u'][i]:.2f} at abort tick"
    released = (r.t["reason"] == "interlock").any()
    return released, "no interlock release (row 18)"


def chk_t15_legal_transitions(r: Run):
    s = r.t["state"]
    change = np.flatnonzero(s[1:] != s[:-1]) + 1
    illegal = [(s[i - 1], s[i]) for i in change
               if (s[i - 1], s[i]) not in LEGAL]
    return not illegal, f"illegal transitions {illegal}"


def chk_t16_crawl_silent_between(r: Run):
    crawl = r.t["state"] == "CRAWL"
    if not crawl.any():
        return False, "no CRAWL ticks (vacuous)"
    bad = crawl & (r.t["u_cmd"] != 0.0)
    return not bad.any(), f"{bad.sum()} nonzero inter-pulse commands in CRAWL"


def chk_t17_drag_released(r: Run, drag_start: int = 1000, drag_end: int = 1500):
    gd = r.s["grab_detect_tick"]
    ok = gd >= 0 and drag_start <= gd <= drag_end + 300
    return ok, f"drag override detected at {gd}"


# ------------------------------------------------------------------ tests
# pattern: one positive + one negative (seeded bug) per tripwire

def _both(tmp_path, scenario, checker, bug, tag, **kw):
    good = run_sim(tmp_path, scenario, tag=f"{tag}_ok")
    ok, msg = checker(good, **kw)
    assert ok, f"positive: {msg}"
    bad = run_sim(tmp_path, scenario, bug=bug, tag=f"{tag}_bug")
    ok_bad, _ = checker(bad, **kw)
    assert not ok_bad, f"negative control vacuous: bug {bug} not caught"


def test_t1_rest_silence(tmp_path):
    _both(tmp_path, SC_REST_HAND, chk_t1_silence, 1, "t1")


def test_t2_no_creep(tmp_path):
    _both(tmp_path, SC_MOVE, chk_t2_no_creep, 2, "t2")


def test_t3_ff_desired_sign(tmp_path):
    _both(tmp_path, SC_MOVE_REV, chk_t3_ff_sign, 3, "t3")


def test_t4_stall_from_progress(tmp_path):
    _both(tmp_path, SC_GRAB, chk_t4_grab_stall, 4, "t4")


def test_t5_gate_escape(tmp_path):
    _both(tmp_path, SC_REST_HAND, chk_t5_offset_adopted, 5, "t5")


def test_t6_grab_releases(tmp_path):
    _both(tmp_path, SC_GRAB, chk_t6_release_torque_free, 6, "t6")


def test_t7_forward_landing(tmp_path):
    for sc, tag in ((SC_MOVE, "t7f"), (SC_MOVE_REV, "t7r"),
                    (SC_SHORT_REV, "t7s")):
        good = run_sim(tmp_path, sc, tag=f"{tag}_ok")
        ok, msg = chk_t7_fwd_landing(good)
        assert ok, f"{tag}: {msg}"
    bad = run_sim(tmp_path, SC_SHORT_REV, bug=7, tag="t7_bug")
    ok_bad, _ = chk_t7_fwd_landing(bad)
    assert not ok_bad, "T7 negative control vacuous"


def test_t8_honest_gave_up(tmp_path):
    # the endgame self-corrects too well to fail naturally (scanned: even
    # +-7 % friction / -35 % m plants land) — T8 is about flag HONESTY, so
    # the budget is pinned to one pulse, which cannot land 3.4 % within tol
    sc = ("p0 30\nduration 8\nconfig impulse_max 1\nat 0.05 target 33.4\n")
    good = run_sim(tmp_path, sc, tag="t8_ok")
    assert good.s["settle_flag"] == "gave_up", \
        f"expected gave_up, got {good.s['settle_flag']}"
    ok, msg = chk_t8_honest_flag(good)
    assert ok, msg
    bad = run_sim(tmp_path, sc, bug=8, tag="t8_bug")
    ok_bad, _ = chk_t8_honest_flag(bad)
    assert not ok_bad, "T8 negative control vacuous"


def test_t9_bounded_activity(tmp_path):
    _both(tmp_path, SC_GRAB_LONG, chk_t9_terminal, 9, "t9")


def test_t10_pulse_domain(tmp_path):
    _both(tmp_path, SC_MOVE, chk_t10_pulse_domain, 10, "t10")


def test_t11_duty_bounded(tmp_path):
    _both(tmp_path, SC_GRAB, chk_t11_duty_bounded, 11, "t11")


def test_t12_observer_input_independence(tmp_path):
    _both(tmp_path, SC_FREEZE, chk_t12_sensor_wins, 12, "t12")


def test_t13_recorder_independence(tmp_path):
    def duty_trace(bug, rec, tag):
        return run_sim(tmp_path, SC_MOVE, bug=bug, recorder=rec,
                       tag=tag).t["u"]
    off = duty_trace(0, 0, "t13_off")
    on = duty_trace(0, 100000, "t13_on")
    tiny = duty_trace(0, 64, "t13_tiny")
    assert np.array_equal(off, on) and np.array_equal(off, tiny)
    bugged = duty_trace(13, 64, "t13_bug")
    assert not np.array_equal(off, bugged), "T13 negative control vacuous"


def test_t14_interlock_supremacy(tmp_path):
    _both(tmp_path, SC_ABORT, chk_t14_same_tick_clamp, 14, "t14")


def test_t15_no_rogue_transitions(tmp_path):
    for sc, tag in ((SC_RETARGET2, "t15a"), (SC_GRAB, "t15b"),
                    (SC_CRAWL_GRAB, "t15c"), (SC_ABORT, "t15d")):
        good = run_sim(tmp_path, sc, tag=f"{tag}_ok")
        ok, msg = chk_t15_legal_transitions(good)
        assert ok, f"{tag}: {msg}"
    bad = run_sim(tmp_path, SC_RETARGET2, bug=15, tag="t15_bug")
    ok_bad, _ = chk_t15_legal_transitions(bad)
    assert not ok_bad, "T15 negative control vacuous"


def test_t16_crawl_zero_between_pulses(tmp_path):
    _both(tmp_path, SC_CRAWL, chk_t16_crawl_silent_between, 16, "t16")


def test_t17_crawl_drag_override(tmp_path):
    _both(tmp_path, SC_CRAWL_DRAG, chk_t17_drag_released, 17, "t17")


def test_crawl_blocking_grab(tmp_path):
    r = run_sim(tmp_path, SC_CRAWL_GRAB, tag="cgrab")
    gd = r.s["grab_detect_tick"]
    assert gd >= 0, "holding hand not detected in CRAWL"
    assert gd - 1000 <= 1500, f"blocking grab took {gd - 1000} ticks"
    after = r.t["u_cmd"][int(gd) + 2:3000]
    assert (after == 0.0).all(), "torque after CRAWL grab"


# --------------------------------------------------- production-build parity

def test_prod_build_identical_and_bug_free(tmp_path):
    a = run_sim(tmp_path, SC_MOVE, binary=RUNNER, tag="par_sil")
    b = run_sim(tmp_path, SC_MOVE, binary=RUNNER_PROD, tag="par_prod")
    assert np.array_equal(a.t["u"], b.t["u"]), "bug hooks not inert at bug=0"
    # in the production build --bug N must be dead: identical trace
    c = run_sim(tmp_path, SC_MOVE, bug=3, binary=RUNNER_PROD, tag="par_bug")
    assert np.array_equal(b.t["u"], c.t["u"]), "bug code alive in prod build"


# --------------------------------------------------------- acceptance smoke

def test_accept_a5_move_time(tmp_path):
    r = run_sim(tmp_path, "p0 20\nat 0.05 target 70\n", tag="a5")
    assert r.s["settle_flag"] == "reached"
    assert (r.s["settled_tick"] - 50) <= 1200, f"{r.s['settled_tick']} ticks"


def test_accept_a7_pull_in(tmp_path):
    for p0, tag in ((47.0, "a7lo"), (53.0, "a7hi")):
        r = run_sim(tmp_path, f"p0 {p0}\nat 0.05 target 50\n", tag=tag)
        assert r.s["settle_flag"] == "reached", tag
        assert (r.s["settled_tick"] - 50) <= 400, f"{tag}: {r.s['settled_tick']}"
        assert r.s["overshoots"] <= 1


def test_accept_a1_lite_perturbed(tmp_path):
    errs = []
    cases = [(15, 75, 0.05, -0.05), (85, 25, -0.06, 0.07), (30, 60, 0.07, 0.07),
             (70, 40, -0.07, -0.07), (10, 90, 0.03, -0.03), (90, 65, 0.07, 0.07)]
    for k, (p0, tgt, df, dr) in enumerate(cases):
        sc = (f"p0 {p0}\nseed {k + 10}\nperturb {df} {dr} "
              f"{0.1 if k % 2 else -0.1} {-0.1 if k % 2 else 0.1}\n"
              f"at 0.05 target {tgt}\n")
        r = run_sim(tmp_path, sc, tag=f"a1_{k}")
        assert r.s["final_state"] == "IDLE", f"case {k} did not settle"
        errs.append(abs(r.s["final_pos"] - r.s["target"]))
    errs = np.array(errs)
    assert np.median(errs) <= 0.5, f"median {np.median(errs):.2f}"
    assert errs.max() <= 1.0, f"max {errs.max():.2f}"


def test_accept_crawl_rate_tracking(tmp_path):
    r = run_sim(tmp_path, SC_CRAWL, tag="crate")
    assert r.s["settle_flag"] == "reached"
    assert r.s["crawl_degraded"] == 0
    crawl = np.flatnonzero(r.t["state"] == "CRAWL")
    dur_s = (crawl[-1] - crawl[0]) / 1000.0
    travel = abs(r.t["pos"][crawl[-1]] - r.t["pos"][crawl[0]])
    rate = travel / dur_s
    assert 10.0 <= rate <= 26.0, f"crawl rate {rate:.1f} %/s for requested 20"
