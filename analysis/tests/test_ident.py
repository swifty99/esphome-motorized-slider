"""Ident L6 + R7/R8 tripwires (kueche-mief-spec.md §4, K2–K5, K7–K9 + T15-
pattern fuzz). K1/K6/K10/K11 belong to the wrapper / hardware.

Pattern of test_sil.py: every tripwire is a checker that must PASS on the
real core and FAIL under its seeded bug (--bug N, compiled only under
SLIDER_SIL_BUGS) — the proof the checker is not vacuous. The ident runs
against a TRUTH plant from the §4 family (sim_runner --ident) and never
sees it: raw ADC counts + its own duty only.
"""

from __future__ import annotations

import csv
import random
import subprocess
from pathlib import Path

import numpy as np
import pytest

from test_sil import RUNNER, build_runner, run_sim  # noqa: F401  (session fixture)

# seeded bugs (firmware/components/slider_servo/sil_bugs.h)
K2_BRACKET_RIG, K3_DIR_FIXED, K4_EARLY_COMMIT, K5_NO_HAND = 20, 21, 22, 23
K7_RIG_DUTY, K8_UNID_FLAG_CLEAR, K9_SINGLE_PROBE, KF_ROGUE_IDENT = 24, 25, 26, 27

BUDGET_TICKS = 120_000  # §3 budget: <= 120 s incl. pauses and repositioning
LAND_TOL = 2.0          # % (Gate answer 6 / K3 / K8)


def run_ident(tmp_path: Path, scenario: str, bug: int = 0, tag: str = "i",
              trace: bool = False):
    sf = tmp_path / f"{tag}.txt"
    sf.write_text(scenario, encoding="utf-8")
    cmd = [str(RUNNER), "--ident", str(sf), "--bug", str(bug)]
    out = tmp_path / f"{tag}_{bug}.csv"
    if trace:
        cmd += ["--out", str(out)]
    proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
    s: dict = {}
    for line in proc.stdout.splitlines():
        if "=" not in line:
            continue
        k, v = line.split("=", 1)
        try:
            s[k] = int(v)
        except ValueError:
            try:
                s[k] = float(v)
            except ValueError:
                s[k] = v
    t = None
    if trace:
        with out.open() as f:
            rows = list(csv.DictReader(f))
        t = {}
        for k in rows[0]:
            col = [r[k] for r in rows]
            t[k] = np.array(col) if k in ("state", "flag", "reason") else \
                np.array([float(x) for x in col])
    return s, t


def moves(s: dict):
    return [(s[f"move{k}_target"], s[f"move{k}_final"], s[f"move{k}_flag"])
            for k in range(1, int(s["moves"]) + 1)]


MOVES_AFTER = ("on DONE 1.0 target 30\non DONE 4.0 target 70\n"
               "on DONE 7.0 target 52\n")


def sc_ident(variant: str = "", p0: float = 50, seed: int = 3, extra: str = "",
             duration: float = 200) -> str:
    return (f"p0 {p0}\nseed {seed}\nduration {duration}\n{variant}\n"
            f"at 0.1 ident\n{extra}")


# --------------------------------------------------------------- the family
# kueche-mief-spec §4: breakaway 0.4–1.25 x nominal, polarity/pot, noise x1–x3,
# backlash <= 2 %, supply ±15 %, breakaway spread. Every one must identify
# within the budget and the core — running on the IDENTIFIED calib — must
# land moves within 2 %.

FAMILY = {
    "nominal": "",
    "fr0.40": "friction_scale 0.4",
    "fr0.50": "friction_scale 0.5",
    "fr0.61": "friction_scale 0.61",
    "fr0.80": "friction_scale 0.8",
    "fr1.15": "friction_scale 1.15",
    "fr1.25": "friction_scale 1.25",
    "pol-": "motor_pol -1",
    "pot+": "pot_inv 0",
    "pol-pot+": "motor_pol -1\npot_inv 0",
    "noise2": "noise 2",
    "noise3": "noise 3",
    "bl1": "backlash 1",
    "bl2": "backlash 2",
    "sup0.85": "supply_scale 0.85",
    "sup1.15": "supply_scale 1.15",
    "spread": "fs_spread 0.03",
    "heavy-mix": "friction_scale 1.15\nnoise 3\nbacklash 2\nfs_spread 0.03",
    "light-mix": ("friction_scale 0.4\nnoise 3\nbacklash 2\nsupply_scale 1.15\n"
                  "fs_spread 0.03\nmotor_pol -1\npot_inv 0"),
}


@pytest.mark.parametrize("name", list(FAMILY))
def test_family_identifies_and_lands(tmp_path, name):
    s, _ = run_ident(tmp_path, sc_ident(FAMILY[name], extra=MOVES_AFTER), tag=name)
    assert s["run1_outcome"] == "ok", f"{name}: {s['run1_reason']}"
    assert s["total_ticks"] <= BUDGET_TICKS, f"{name}: {s['total_ticks']} ticks"
    assert s["dir_sign"] == s["truth_dir_sign"], f"{name}: polarity"
    assert s["final_identified"] == 1 and s["final_derived"] == 1, \
        "core not running on the identified calib"
    for tgt, fin, flag in moves(s):
        assert abs(fin - tgt) <= LAND_TOL, f"{name}: {tgt} -> {fin:.2f} ({flag})"


@pytest.mark.parametrize("name,variant,reason", [
    # honest limits of the family (findings doc): FAIL, never a bad stamp
    ("edge-fr1.25-spread", "friction_scale 1.25\nnoise 3\nbacklash 2\nfs_spread 0.03",
     None),
    ("fr1.1-sup0.85", "friction_scale 1.1\nsupply_scale 0.85", "plant_implausible"),
    ("fr1.25-sup0.85", "friction_scale 1.25\nsupply_scale 0.85", "no_motion"),
])
def test_family_limits_fail_honestly(tmp_path, name, variant, reason):
    s, _ = run_ident(tmp_path, sc_ident(variant), tag=name)
    if s["run1_outcome"] == "ok":
        assert reason is None, f"{name}: expected FAIL({reason}), got ok"
        return
    assert s["run1_outcome"] == "failed"
    if reason is not None:
        assert s["run1_reason"] == reason, f"{name}: {s['run1_reason']}"
    assert s["nvs_hash_start"] == s["nvs_hash_end"]
    assert s["run1_post_u_max"] == 0.0


# ---------------------------------------------------------------------- K2
# Bracket start independent of rig constants: on a breakaway-0.35 unit
# (friction x0.5, F_s 0.20–0.36) nothing moves before the first bracket
# stage: the first hold is at 0.15 and stays still, I4 is still, and the I6
# stall (derived, F_s_min - 0.10) does not move. Negative: bracket from 0.60
# and a fixed 0.45 stall.

SC_K2 = sc_ident("friction_scale 0.5", seed=5, duration=150)


def chk_k2(s, t):
    ident = t["ident"]
    u = np.abs(t["u"])
    pos = t["pos_frame"]
    i5 = np.flatnonzero((ident == 5) & (u > 0))
    # first constant-duty segment of >= 100 ticks inside I5 = the first hold
    first = None
    k = 0
    while k < len(i5):
        j = k
        while j + 1 < len(i5) and i5[j + 1] == i5[j] + 1 and u[i5[j + 1]] == u[i5[k]]:
            j += 1
        if j - k + 1 >= 100:
            first = (i5[k], i5[j])
            break
        k = j + 1
    if first is None:
        return False, "no bracket hold found"
    a, b = first
    duty = float(u[a])
    moved = float(np.ptp(pos[a:b + 1]))
    if abs(duty - 0.15) > 1e-6:
        return False, f"first bracket stage at {duty:.2f}, not 0.15"
    if moved >= 1.0:
        return False, f"slider moved {moved:.2f} % in the first stage"
    i4 = ident == 4
    if float(np.ptp(pos[i4])) >= 0.5:
        return False, "motion during I4"
    i6 = ident == 6
    if not i6.any():
        return False, f"no I6 ({s['run1_reason']})"
    if float(np.ptp(pos[i6])) >= 1.0:
        return False, f"I6 stall moved {np.ptp(pos[i6]):.2f} %"
    return True, "ok"


def test_k2_bracket_independent_of_rig_constants(tmp_path):
    s, t = run_ident(tmp_path, SC_K2, tag="k2", trace=True)
    ok, msg = chk_k2(s, t)
    assert ok, f"positive: {msg}"
    assert s["run1_outcome"] == "ok"
    assert s["truth_fs_p95"] < 0.40  # the unit really is a ~0.35 unit
    sb, tb = run_ident(tmp_path, SC_K2, bug=K2_BRACKET_RIG, tag="k2b", trace=True)
    ok_bad, _ = chk_k2(sb, tb)
    assert not ok_bad, "K2 negative control vacuous"


# ---------------------------------------------------------------------- K3
# All 4 polarity x pot combinations (and both rail starts, row 6): after the
# ident a move lands within 2 %. Negative: dir_sign fixed +1.

K3_CASES = [(p0, pol, inv) for p0 in (50, 8) for pol in (1, -1) for inv in (1, 0)]


def k3_all_land(tmp_path, bug):
    bad = []
    for p0, pol, inv in K3_CASES:
        sc = sc_ident(f"motor_pol {pol}\npot_inv {inv}", p0=p0, seed=5,
                      extra="on DONE 1.0 target 25\non DONE 4.0 target 75\n")
        s, _ = run_ident(tmp_path, sc, bug=bug, tag=f"k3_{p0}_{pol}_{inv}_{bug}")
        if s["run1_outcome"] != "ok":
            bad.append((p0, pol, inv, s["run1_reason"]))
            continue
        for tgt, fin, _ in moves(s):
            if abs(fin - tgt) > LAND_TOL:
                bad.append((p0, pol, inv, tgt, fin))
    return bad


def test_k3_polarity_and_pot_learned(tmp_path):
    bad = k3_all_land(tmp_path, 0)
    assert not bad, f"positive: {bad}"
    assert k3_all_land(tmp_path, K3_DIR_FIXED), "K3 negative control vacuous"


# ---------------------------------------------------------------------- K4
# Every FAIL (and ABORT) cause of §3 leaves the previous NVS stamp
# bit-identical and the duty 0. NVS = the runner's blob, written only when
# commit_seq changes (the wrapper's main-task write). Negative: publish
# before I9.

K4_CASES = {
    "adc_flat": ("", "at 0.0 adc flat\n", "failed"),
    "adc_rail": ("", "at 0.0 adc rail\n", "failed"),
    "adc_noise": ("", "at 0.0 adc stuck 4000\n", "failed"),
    "thermal": ("", "at 0.05 heat 0.7\n", "failed"),
    "supply": ("supply_adc 5.0\nsupply_window 4.5 5.5\nsupply_scale 0.8", "", "failed"),
    "no_motion": ("motor_pol 0", "", "failed"),
    "span": ("pot_raw 3000 5000", "", "failed"),
    "no_breakaway": ("fs_patch 80 90 0.97", "", "failed"),
    "plant_implausible": ("", "on I7 0 friction 1.3\n", "failed"),
    "table_implausible": ("", "on I8 0 friction 1.35\n", "failed"),
    "watchdog": ("", "on I5 0 grab 400\n", "failed"),
    # ABORT causes keep the stamp too (row 26/27/28/30/32)
    "cmd": ("", "on I5 3 abort_ident\n", "aborted"),
    "hand": ("", "on I8 1.0 drag_rel -10 0.2\n", "aborted"),
    "interlock": ("", "on I5 3 abort on\n", "aborted"),
    "supply_abort": ("supply_adc 5.0\nsupply_window 4.5 5.5", "on I5 3 supply 0.8\n",
                     "aborted"),
    "fault": ("", "on I5 3 nan\n", "aborted"),
}


def chk_k4(s, outcome, reason):
    if s["run1_outcome"] != outcome:
        return False, f"outcome {s['run1_outcome']} ({s['run1_reason']})"
    if s["run1_reason"] != reason:
        return False, f"reason {s['run1_reason']} != {reason}"
    if s["nvs_hash_start"] != s["nvs_hash_end"] or s["nvs_writes"] != 0:
        return False, "previous NVS stamp modified"
    if s["run1_post_u_max"] != 0.0:
        return False, f"duty {s['run1_post_u_max']} after the end"
    return True, "ok"


def _k4_reason(name):
    return "supply" if name == "supply_abort" else name


@pytest.mark.parametrize("name", list(K4_CASES))
def test_k4_fail_keeps_previous_stamp(tmp_path, name):
    variant, extra, outcome = K4_CASES[name]
    dur = 300 if name == "watchdog" else 200
    s, _ = run_ident(tmp_path, sc_ident(variant, seed=4, extra=extra, duration=dur),
                     tag=f"k4_{name}")
    ok, msg = chk_k4(s, outcome, _k4_reason(name))
    assert ok, f"{name}: {msg}"


def test_k4_negative_control_early_commit(tmp_path):
    # the late failures (after I5) must catch the pre-I9 NVS write
    broken = []
    for name in ("plant_implausible", "table_implausible", "hand"):
        variant, extra, outcome = K4_CASES[name]
        s, _ = run_ident(tmp_path, sc_ident(variant, seed=4, extra=extra),
                         bug=K4_EARLY_COMMIT, tag=f"k4b_{name}")
        ok, _ = chk_k4(s, outcome, _k4_reason(name))
        broken.append(not ok)
    assert all(broken), f"K4 negative control vacuous: {broken}"


# ---------------------------------------------------------------------- K5
# Hand during ident (motion at duty 0) -> ABORT(hand), duty 0 from the
# decision on (the ABORT tick already outputs 0; I0 on the next tick).
# Negative: row 27 deleted.

K5_CASES = {
    "I4": "on I4 0.5 drag_rel -15 0.3\n",   # quiet phase at rail B
    "I8": "on I8 1.0 drag_rel -10 0.2\n",   # between impulses
}


def chk_k5(s, t, phase, delay):
    if s["run1_outcome"] != "aborted" or s["run1_reason"] != "hand":
        return False, f"{s['run1_outcome']}/{s['run1_reason']}"
    drag = s[f"run1_enter_{phase}"] + int(delay * 1000)
    end = s["run1_end"]
    if not (drag < end <= drag + 400):
        return False, f"detected {end - drag} ticks after the hand"
    u = np.abs(t["u"])
    # from the abort decision (end - 1) on: exactly zero
    if (u[end - 1:] != 0.0).any():
        return False, "torque after the hand ABORT"
    return True, "ok"


@pytest.mark.parametrize("phase", list(K5_CASES))
def test_k5_hand_aborts(tmp_path, phase):
    extra = K5_CASES[phase]
    delay = 0.5 if phase == "I4" else 1.0
    sc = sc_ident(seed=4, extra=extra)
    s, t = run_ident(tmp_path, sc, tag=f"k5_{phase}", trace=True)
    ok, msg = chk_k5(s, t, int(phase[1]), delay)
    assert ok, f"positive {phase}: {msg}"
    sb, tb = run_ident(tmp_path, sc, bug=K5_NO_HAND, tag=f"k5b_{phase}", trace=True)
    ok_bad, _ = chk_k5(sb, tb, int(phase[1]), delay)
    assert not ok_bad, f"K5 negative control vacuous ({phase})"


# ---------------------------------------------------------------------- K7
# R7 on a breakaway-0.43 unit: every core pulse <= impulse_duty (= the
# derived clamp(F_s_p95 + 0.10, 0.60, 0.95)), u_max > F_s_p95, and a grab is
# still detected. Negative: impulse_duty 0.8 / u_max 0.9 fixed.

SC_K7 = sc_ident("friction_scale 0.61", seed=5, extra=(
    "on DONE 1.0 target 30\non DONE 4.0 target 33\n"
    "on DONE 7.0 target 80\non DONE 7.15 grab 1.5\n"))


def chk_k7(s):
    p95 = s["fs_p95"]
    want = min(max(p95 + 0.10, 0.60), 0.95)
    if abs(s["impulse_duty"] - want) > 1e-3:
        return False, f"impulse_duty {s['impulse_duty']} != derived {want:.3f}"
    for k in range(1, int(s["moves"]) + 1):
        if s[f"move{k}_pulse_u_max"] > s["impulse_duty"] + 1e-6:
            return False, f"move {k}: pulse {s[f'move{k}_pulse_u_max']}"
    if not (s["u_max"] > p95 and s["u_max"] > s["truth_fs_p95"]):
        return False, f"u_max {s['u_max']} <= F_s_p95"
    grab_t = s["run1_end"] + 7150
    gd = s["grab_detect_post"]
    if not (gd >= 0 and gd - grab_t <= 600):
        return False, f"grab not detected ({gd}, grab at {grab_t})"
    return True, "ok"


def test_k7_derived_impulse_and_umax(tmp_path):
    s, _ = run_ident(tmp_path, SC_K7, tag="k7")
    assert s["truth_fs_p95"] < 0.46  # really a ~0.43 unit
    ok, msg = chk_k7(s)
    assert ok, f"positive: {msg}"
    for tgt, fin, _ in moves(s)[:2]:
        assert abs(fin - tgt) <= LAND_TOL
    sb, _ = run_ident(tmp_path, SC_K7, bug=K7_RIG_DUTY, tag="k7b")
    ok_bad, _ = chk_k7(sb)
    assert not ok_bad, "K7 negative control vacuous"


# ---------------------------------------------------------------------- K8
# R8 unidentified: moves land within 2 % WITHOUT the cruise path (no MOVE /
# CRAWL / GLIDE tick), the status flag is set every tick. Negative: flag
# cleared after the first move. Legacy runner (tick(z_pct)) + 'unidentified'.

K8_PLANTS = {"nominal": "", "light0.61": "perturb -0.39 -0.39 0 0",
             "band7": "perturb 0.07 -0.07 0.1 -0.1"}


def chk_k8(r):
    st = r.t["state"]
    if np.isin(st, ["MOVE", "CRAWL", "GLIDE"]).any():
        return False, "cruise path used while unidentified"
    if (r.t["unid"] != 1).any():
        return False, f"{int((r.t['unid'] != 1).sum())} ticks without the flag"
    return True, "ok"


@pytest.mark.parametrize("plant", list(K8_PLANTS))
def test_k8_unidentified_throttled(tmp_path, plant):
    for k, (p0, tgt) in enumerate(((40, 70), (40, 12), (40, 88), (60, 58))):
        sc = (f"p0 {p0}\nunidentified\n{K8_PLANTS[plant]}\nduration 30\nrest 1.0\n"
              f"at 0.05 target {tgt}\nat 0.05 target {tgt} speed 20\n")
        r = run_sim(tmp_path, sc, tag=f"k8_{plant}_{k}")
        ok, msg = chk_k8(r)
        assert ok, f"{plant} {p0}->{tgt}: {msg}"
        assert r.s["final_state"] == "IDLE"
        assert abs(r.s["final_pos"] - tgt) <= LAND_TOL, \
            f"{plant} {p0}->{tgt}: {r.s['final_pos']:.2f} ({r.s['settle_flag']})"


def test_k8_negative_control_flag_cleared(tmp_path):
    sc = "p0 40\nunidentified\nduration 30\nrest 1.0\nat 0.05 target 70\n"
    r = run_sim(tmp_path, sc, bug=K8_UNID_FLAG_CLEAR, tag="k8b")
    ok, _ = chk_k8(r)
    assert not ok, "K8 negative control vacuous"


# ---------------------------------------------------------------------- K9
# Two idents back to back: breakaway ±0.03 on every point, b ±15 %, impulse
# travels ±10 %. Honest scope (findings doc §K9): impulse check on widths
# >= 10 ms only (3/5 ms travel 0.1–0.4 % sits at the noise/position floor),
# and on the spread-free plant; on the rig-realistic spread plant the strict
# statement is RED (xfail below) and the negative control is shown against a
# weakened, documented bound.

def k9_pair(tmp_path, variant, seed, bug, tag):
    sc = sc_ident(variant, seed=seed, duration=300, extra="on DONE 1.0 ident\n")
    s, _ = run_ident(tmp_path, sc, bug=bug, tag=tag)
    assert s["run1_outcome"] == "ok" and s["run2_outcome"] == "ok", \
        f"{tag}: {s.get('run1_reason')} / {s.get('run2_reason')}"
    fs = [abs(s[f"run1_fs_{d}_{k}"] - s[f"run2_fs_{d}_{k}"])
          for d in ("fwd", "rev") for k in range(7)]
    b = max(abs(s[f"run1_b_{d}"] / s[f"run2_b_{d}"] - 1) for d in ("fwd", "rev"))
    imp = [abs(s[f"run1_imp_{d}_{k}"] / s[f"run2_imp_{d}_{k}"] - 1)
           for d in ("fwd", "rev") for k in range(2, 6)]
    return np.array(fs), b, max(imp)


def test_k9_repeatability_spread_free(tmp_path):
    for seed in (3, 5, 7, 11):
        fs, b, imp = k9_pair(tmp_path, "", seed, 0, f"k9_{seed}")
        assert fs.max() <= 0.03 + 1e-3, f"seed {seed}: breakaway {fs.max():.3f}"
        assert b <= 0.15, f"seed {seed}: b {b:.3f}"
        assert imp <= 0.10, f"seed {seed}: impulse {imp:.3f}"


SPREAD_SEEDS = range(1, 9)


def k9_spread_stats(tmp_path, bug):
    fs_all, b_max, imp_max = [], 0.0, 0.0
    for seed in SPREAD_SEEDS:
        fs, b, imp = k9_pair(tmp_path, "fs_spread 0.03", seed, bug, f"k9s_{seed}_{bug}")
        fs_all += fs.tolist()
        b_max, imp_max = max(b_max, b), max(imp_max, imp)
    return np.array(fs_all), b_max, imp_max


def chk_k9_weak(fs, b):
    # WEAKENED bound (documented): >= 90 % of points within ±0.03, none beyond
    # ±0.045, b ±15 %
    inside = float((fs <= 0.03 + 1e-3).mean())
    return inside >= 0.90 and fs.max() <= 0.045 + 1e-3 and b <= 0.15, \
        f"inside {inside:.2f}, max {fs.max():.3f}, b {b:.3f}"


def test_k9_spread_plant_weak_bound_and_negative(tmp_path):
    fs, b, _ = k9_spread_stats(tmp_path, 0)
    ok, msg = chk_k9_weak(fs, b)
    assert ok, f"positive: {msg}"
    fsb, bb, _ = k9_spread_stats(tmp_path, K9_SINGLE_PROBE)
    ok_bad, _ = chk_k9_weak(fsb, bb)
    assert not ok_bad, "K9 negative control vacuous"


@pytest.mark.xfail(strict=True, reason="K9 strict is red on the rig-realistic "
                   "breakaway spread: 2 reps cannot hold ±0.03 on every point "
                   "and ±10 % on every impulse (findings doc §K9, question Q4)")
def test_k9_strict_on_spread_plant(tmp_path):
    fs, b, imp = k9_spread_stats(tmp_path, 0)
    assert fs.max() <= 0.03 + 1e-3 and b <= 0.15 and imp <= 0.10


# ----------------------------------------------------- T15 pattern for L6
# (state, trigger) pairs outside §3 are no-ops: random triggers in all
# states; only §3 transitions may appear, the controller stays IDENT while a
# run is active and every set_target during a run is rejected.

IDENT_LEGAL = {(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 7), (7, 8),
               (8, 9), (9, 0), (10, 0), (11, 0)}
IDENT_LEGAL |= {(k, 10) for k in (1, 2, 3, 4, 5, 6, 7, 8)}   # FAIL rows + 29
IDENT_LEGAL |= {(k, 11) for k in range(1, 10)}               # ABORT rows


def fuzz_scenario(seed: int, duration: float = 600.0, gap=(0.3, 6.0)) -> str:
    rnd = random.Random(seed)
    lines = [f"p0 50\nseed {seed}\nduration {duration}\nat 0.1 ident\n",
             "on I5 5 ident\n"]  # guaranteed CMD Ident inside I5
    t = 0.5
    while t < duration - 5:
        t += rnd.uniform(*gap)
        r = rnd.random()
        if r < 0.35:
            lines.append(f"at {t:.3f} ident\n")
        elif r < 0.55:
            lines.append(f"at {t:.3f} target {rnd.uniform(5, 95):.1f}\n")
        elif r < 0.60:
            lines.append(f"at {t:.3f} abort_ident\n")
        elif r < 0.65:
            lines.append(f"at {t:.3f} release\n")
        elif r < 0.70:
            lines.append(f"at {t:.3f} abort on\nat {t + 0.05:.3f} abort off\n")
        elif r < 0.80:
            lines.append(f"at {t:.3f} drag_rel {rnd.choice((-1, 1)) * 8} 0.2\n")
        elif r < 0.85:
            lines.append(f"at {t:.3f} nan\n")
        elif r < 0.90:
            lines.append(f"at {t:.3f} step {rnd.uniform(-3, 3):.2f}\n")
        elif r < 0.95:
            lines.append(f"at {t:.3f} adc flat\nat {t + 0.3:.3f} adc ok\n")
        # else: nothing (quiet stretch)
    return "".join(lines)


def chk_fuzz(s):
    pairs = []
    for item in str(s["ident_transitions"]).split(","):
        if not item:
            continue
        ab, _ = item.split(":")
        a, b = ab.split(">")
        pairs.append((int(a), int(b)))
    illegal = [p for p in pairs if p not in IDENT_LEGAL]
    if illegal:
        return False, f"illegal ident transitions {illegal}"
    if s["ctrl_violation"] != 0:
        return False, "controller left IDENT during a run"
    if s["accepted_during"] != 0:
        return False, "set_target accepted during a run"
    return True, f"{len(pairs)} distinct transitions"


def test_ident_fuzz_no_rogue_transitions(tmp_path):
    seen = set()
    # dense to sparse: dense fuzz hits the early phases from every side,
    # sparse fuzz lets runs reach I6..I9 and complete
    for seed, gap in ((1, (0.3, 6.0)), (2, (2.0, 25.0)), (3, (5.0, 60.0)),
                      (4, (10.0, 90.0))):
        s, _ = run_ident(tmp_path, fuzz_scenario(seed, gap=gap), tag=f"fz{seed}")
        ok, msg = chk_fuzz(s)
        assert ok, f"seed {seed}: {msg}"
        assert s["starts_during"] > 0 and s["rejected_during"] > 0  # non-vacuous
        seen |= {item.split(":")[0] for item in str(s["ident_transitions"]).split(",")
                 if item}
    assert len(seen) >= 12, f"fuzz too narrow: {sorted(seen)}"
    sb, _ = run_ident(tmp_path, fuzz_scenario(1), bug=KF_ROGUE_IDENT, tag="fzb")
    ok_bad, _ = chk_fuzz(sb)
    assert not ok_bad, "fuzz negative control vacuous"


# ------------------------------------------------- set_target during ident

def test_set_target_rejected_during_ident(tmp_path):
    s, t = run_ident(tmp_path, sc_ident(extra="on I5 2 target 80\n"), tag="rej",
                     trace=True)
    assert s["move1_status"] == 1  # CmdStatus::REJECTED_IDENT
    assert s["run1_outcome"] == "ok"  # and the ident is undisturbed
