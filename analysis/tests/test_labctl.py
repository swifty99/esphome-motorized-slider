"""Unit tests for the pure parts of labctl -- no hardware, no network."""

import numpy as np
import pandas as pd
import pytest

from slideranalysis.labctl import (
    BLIP_DUTY,
    Check,
    LabctlError,
    RunFacts,
    assemble_preflight,
    assemble_verdict,
    blip_direction,
    blip_params,
    detect_reboot,
    estimate_duration_s,
    evaluate_blip,
    evaluate_position,
    evaluate_scope_shunt,
    evaluate_scope_wiper,
    load_state,
    save_state,
    validate_params,
)

# ---------------------------------------------------------------------------
# params validation
# ---------------------------------------------------------------------------


def test_validate_params_defaults_empty():
    assert validate_params("noise_floor", None) == {}
    assert validate_params("noise_floor", "") == {}
    assert validate_params("noise_floor", "   ") == {}


def test_validate_params_parses_object():
    params = validate_params("pwm_coupling", '{"duty":0.2,"dir":"fwd","mode":"fast"}')
    assert params == {"duty": 0.2, "dir": "fwd", "mode": "fast"}


def test_validate_params_rejects_unknown_experiment():
    with pytest.raises(LabctlError, match="unknown experiment"):
        validate_params("warp_drive", None)


def test_validate_params_rejects_bad_json():
    with pytest.raises(LabctlError, match="not valid JSON"):
        validate_params("noise_floor", "{duty: 0.2}")


def test_validate_params_rejects_non_object():
    with pytest.raises(LabctlError, match="JSON object"):
        validate_params("noise_floor", "[1, 2]")


# ---------------------------------------------------------------------------
# duration estimation (mirrors the firmware program builder)
# ---------------------------------------------------------------------------


def test_estimate_noise_floor_default():
    # 10 s settle + 0.5 s recording tail
    assert estimate_duration_s("noise_floor", {}) == pytest.approx(10.5)


def test_estimate_noise_floor_param_override():
    assert estimate_duration_s("noise_floor", {"duration_s": 3}) == pytest.approx(3.5)


def test_estimate_pwm_coupling_default():
    # 0.3 settle + 10 s hold + 0.3 settle + tail
    assert estimate_duration_s("pwm_coupling", {}) == pytest.approx(11.1)


def test_estimate_duty_ladder_default():
    # 12 rungs x (2 s + 0.5 s) + tail
    assert estimate_duration_s("duty_ladder", {}) == pytest.approx(30.5)


def test_estimate_duty_ladder_rejects_bad_ladder():
    with pytest.raises(LabctlError):
        estimate_duration_s("duty_ladder", {"step": 0})


def test_estimate_step_response_default():
    # goto worst case 4 s + 0.5 + 5 s hold + 0.5 + tail
    assert estimate_duration_s("step_response", {}) == pytest.approx(10.5)


def test_estimate_reversal_approach_ms():
    # per cycle: goto 4 + 0.5 + approach + 0.6 + 1.2 + 0.5
    default = estimate_duration_s("reversal", {"cycles": 1})
    assert default == pytest.approx(4 + 0.5 + 0.6 + 0.6 + 1.2 + 0.5 + 0.5)
    longer = estimate_duration_s("reversal", {"cycles": 1, "approach_ms": 1500})
    assert longer == pytest.approx(default + 0.9)


def test_estimate_breakaway_grid_scales_with_grid():
    # per trial: goto 4 + 0.5 + breakaway 3 + hold 0.3 + 0.3 = 8.1 s
    one = estimate_duration_s(
        "breakaway_grid", {"positions": [50], "repeats": 1, "dir": "fwd"})
    assert one == pytest.approx(8.1 + 0.5)
    both_dirs = estimate_duration_s(
        "breakaway_grid", {"positions": [50], "repeats": 1})
    assert both_dirs == pytest.approx(2 * 8.1 + 0.5)


def test_estimate_covers_all_known_experiments():
    from slideranalysis.labctl import KNOWN_EXPERIMENTS

    for name in KNOWN_EXPERIMENTS:
        assert estimate_duration_s(name, {}) > 0


# ---------------------------------------------------------------------------
# reboot detection + state file
# ---------------------------------------------------------------------------


def test_detect_reboot_first_contact():
    assert detect_reboot(None, uptime_s=100.0, now_ts=1000.0) is None
    assert detect_reboot({}, uptime_s=100.0, now_ts=1000.0) is None


def test_detect_reboot_continuous_uptime_is_quiet():
    prev = {"uptime_s": 100.0, "ts": 1000.0}
    # 50 s later, uptime grew by ~50 s (within the 10 s sensor quantization)
    assert detect_reboot(prev, uptime_s=145.0, now_ts=1050.0) is None


def test_detect_reboot_uptime_lower_than_last_seen():
    prev = {"uptime_s": 500.0, "ts": 1000.0}
    msg = detect_reboot(prev, uptime_s=20.0, now_ts=1030.0)
    assert msg is not None and "REBOOT" in msg


def test_detect_reboot_recovered_past_previous_uptime():
    # rebooted 10 min ago, ran up to 600 s -- higher than prev 500 s, but far
    # below the ~4100 s continuous operation would predict
    prev = {"uptime_s": 500.0, "ts": 1000.0}
    msg = detect_reboot(prev, uptime_s=600.0, now_ts=1000.0 + 3600.0)
    assert msg is not None and "REBOOT" in msg


def test_detect_reboot_slack_absorbs_sensor_quantization():
    prev = {"uptime_s": 100.0, "ts": 1000.0}
    # uptime sensor updates every 10 s: reading may lag by up to ~10-20 s
    assert detect_reboot(prev, uptime_s=85.0, now_ts=1005.0) is None


def test_detect_reboot_ignores_clock_weirdness_and_bad_state():
    assert detect_reboot({"uptime_s": 100.0, "ts": 2000.0},
                         uptime_s=5.0, now_ts=1000.0) is None  # elapsed < 0
    assert detect_reboot({"uptime_s": "junk", "ts": 0}, 5.0, 1.0) is None


def test_state_file_roundtrip(tmp_path):
    path = tmp_path / "state.json"
    assert load_state(path) is None
    save_state(1234.0, 1_700_000_000.0, "192.0.2.1", path=path)
    state = load_state(path)
    assert state["uptime_s"] == 1234.0
    assert state["ts"] == 1_700_000_000.0
    assert state["host"] == "192.0.2.1"


def test_state_file_corrupt_returns_none(tmp_path):
    path = tmp_path / "state.json"
    path.write_text("{not json", encoding="utf-8")
    assert load_state(path) is None


# ---------------------------------------------------------------------------
# verdict assembly
# ---------------------------------------------------------------------------


def _clean_facts(**kw) -> RunFacts:
    base = dict(experiment="noise_floor", n_records=10500, tick_passed=True)
    base.update(kw)
    return RunFacts(**base)


def test_verdict_pass_clean_run():
    level, reason = assemble_verdict(_clean_facts())
    assert level == "PASS"
    assert "10500 records" in reason


def test_verdict_fail_on_tickhealth():
    facts = _clean_facts(tick_passed=False, tick_reasons=("2 missed tick(s)",))
    level, reason = assemble_verdict(facts)
    assert level == "FAIL"
    assert "tickhealth" in reason and "missed" in reason


def test_verdict_fail_on_abort_and_thermal():
    assert assemble_verdict(_clean_facts(aborted=True))[0] == "FAIL"
    assert assemble_verdict(_clean_facts(ticks_thermal=7))[0] == "FAIL"
    assert assemble_verdict(_clean_facts(ticks_abort=3))[0] == "FAIL"


def test_verdict_fail_on_empty_or_unfetched():
    assert assemble_verdict(_clean_facts(n_records=0))[0] == "FAIL"
    assert assemble_verdict(_clean_facts(fetched=False))[0] == "FAIL"
    assert assemble_verdict(_clean_facts(player_returned_idle=False))[0] == "FAIL"


def test_verdict_fail_on_stale_dump_from_refused_run():
    # run_experiment refuses silently over the API -> the fetch grabs the
    # previous run's buffer; the dump/fired mismatch must FAIL loudly
    facts = _clean_facts(experiment="noise_floor", expected_experiment="dither")
    level, reason = assemble_verdict(facts)
    assert level == "FAIL"
    assert "refused" in reason
    ok = _clean_facts(experiment="dither", expected_experiment="dither")
    assert assemble_verdict(ok)[0] == "PASS"


def test_verdict_endzone_marker_is_passive_not_warn():
    # since 2026-07-17 the end-zone flag is a passive data marker (no torque
    # cut) -- it must not cost the PASS, but the count stays visible
    level, reason = assemble_verdict(
        _clean_facts(experiment="pwm_coupling", ticks_endzone=42))
    assert level == "PASS"
    assert "endzone marker" in reason and "42" in reason


def test_verdict_endzone_marker_uniform_for_step_response():
    level, reason = assemble_verdict(
        _clean_facts(experiment="step_response", ticks_endzone=900))
    assert level == "PASS"
    assert "endzone marker" in reason and "900" in reason


def test_verdict_warn_on_missing_scope():
    facts = _clean_facts(scope_expected=True, scope_saved=False)
    assert assemble_verdict(facts)[0] == "WARN"
    facts = _clean_facts(scope_expected=True, scope_saved=True)
    assert assemble_verdict(facts)[0] == "PASS"


def test_verdict_fail_beats_warn():
    facts = _clean_facts(aborted=True, ticks_endzone=5)
    level, reason = assemble_verdict(facts)
    assert level == "FAIL"


def test_verdict_reason_is_single_line():
    facts = _clean_facts(aborted=True, ticks_thermal=2, tick_passed=False,
                         tick_reasons=("a", "b"))
    _level, reason = assemble_verdict(facts)
    assert "\n" not in reason


# ---------------------------------------------------------------------------
# preflight: blip geometry
# ---------------------------------------------------------------------------


def test_blip_direction_away_from_nearer_endstop():
    assert blip_direction(20.0) == 1    # near 0 -> drive fwd (up)
    assert blip_direction(97.8) == -1   # near 100 -> drive bwd (down)
    assert blip_direction(50.0) == -1


def test_blip_params_single_short_pulse():
    p = blip_params(-1)
    assert p["bias"] == -BLIP_DUTY
    assert p["amp"] == 0.0                # amp 0 = pure hold, no actual dither
    assert 0.08 <= p["duration_s"] <= 0.12  # 80-120 ms window


# ---------------------------------------------------------------------------
# preflight: position sanity
# ---------------------------------------------------------------------------


def test_position_pass_mid_travel_stable():
    check = evaluate_position([48.2, 48.3, 48.2, 48.1, 48.2])
    assert check.status == "PASS"


def test_position_warn_in_end_zone():
    check = evaluate_position([97.8, 97.7, 97.8, 97.9])
    assert check.status == "WARN"
    assert "end zone" in check.detail and "passive marker" in check.detail


def test_position_fail_out_of_range_and_unstable():
    assert evaluate_position([99.6, 99.7, 99.6, 99.7]).status == "FAIL"
    assert evaluate_position([0.2, 0.3, 0.2, 0.3]).status == "FAIL"
    assert evaluate_position([50.0, 52.0, 48.0, 51.0]).status == "FAIL"  # std > 0.5


def test_position_fail_too_few_samples():
    assert evaluate_position([50.0]).status == "FAIL"
    assert evaluate_position([]).status == "FAIL"


# ---------------------------------------------------------------------------
# preflight: actuation blip evaluation
# ---------------------------------------------------------------------------


def _blip_df(*, moved_counts=200.0, applied=700, endzone=0, thermal=0):
    """Synthetic blip recording: 300 ms settle, 100 ms pulse, 300 ms settle.

    ``endzone`` ticks are the passive marker (2026-07-17): duty stays applied.
    ``thermal`` ticks simulate a real cut: duty forced to 0.
    """
    n = 700
    duty = np.zeros(n, dtype=int)
    duty[300:400] = applied
    adc = np.full(n, 4000.0)
    adc[350:] -= moved_counts  # fader moved during/after the pulse
    ez = np.zeros(n, dtype=bool)
    if endzone:
        ez[300:300 + endzone] = True  # marker only, torque NOT cut
    th = np.zeros(n, dtype=bool)
    if thermal:
        th[300:300 + thermal] = True
        duty[300:400] = 0  # thermal interlock forces applied duty to 0
    return pd.DataFrame({
        "t_ms": np.arange(n, dtype=float),
        "adc_raw": adc,
        "duty_milli": duty,
        "ilk_endzone": ez,
        "ilk_thermal": th,
        "abort": np.zeros(n, dtype=bool),
    })


def test_blip_pass_moves_and_applies():
    check, delta = evaluate_blip(_blip_df())
    assert check.status == "PASS"
    assert delta == pytest.approx(200.0)
    assert "no interlocks" in check.detail


def test_blip_fail_frozen_adc():
    check, _delta = evaluate_blip(_blip_df(moved_counts=10.0))  # 0.13 % < 0.5 %
    assert check.status == "FAIL"
    assert "frozen ADC" in check.detail


def test_blip_fail_torque_never_applied():
    check, _delta = evaluate_blip(_blip_df(applied=0, moved_counts=0.0))
    assert check.status == "FAIL"
    assert "never reached the bridge" in check.detail


def test_blip_fail_interlock_cut():
    check, _delta = evaluate_blip(_blip_df(thermal=100, moved_counts=0.0))
    assert check.status == "FAIL"
    assert "interlock" in check.detail


def test_blip_endzone_marker_does_not_fail():
    # passive marker ticks with normal movement must not cost the check
    check, delta = evaluate_blip(_blip_df(endzone=100))
    assert check.status == "PASS"
    assert delta == pytest.approx(200.0)


def test_blip_fail_empty():
    check, _delta = evaluate_blip(_blip_df().iloc[0:0])
    assert check.status == "FAIL"


# ---------------------------------------------------------------------------
# preflight: scope cross-checks
# ---------------------------------------------------------------------------


def test_scope_wiper_pass_consistent_swing():
    # firmware delta 200 counts ~ 63 mV; scope sees ~60 mV -> ratio ~1
    wiper = np.concatenate([np.full(500, 1.40), np.full(500, 1.46)])
    assert evaluate_scope_wiper(wiper, adc_delta_counts=200.0).status == "PASS"


def test_scope_wiper_fail_floating_track():
    # tonight's failure: swing ~1/50 of the real one
    wiper = np.concatenate([np.full(500, 1.400), np.full(500, 1.4013)])
    check = evaluate_scope_wiper(wiper, adc_delta_counts=200.0)
    assert check.status == "FAIL"
    assert "wrong track" in check.detail


def test_scope_wiper_fail_no_reference():
    assert evaluate_scope_wiper(np.zeros(100), adc_delta_counts=0.0).status == "FAIL"


def test_scope_shunt_envelope():
    flat = np.full(1000, 0.001)
    assert evaluate_scope_shunt(flat).status == "FAIL"
    pulse = flat.copy()
    pulse[400:500] = 0.080  # ~0.4 A through 0.2 R
    assert evaluate_scope_shunt(pulse).status == "PASS"


# ---------------------------------------------------------------------------
# preflight: verdict assembly
# ---------------------------------------------------------------------------


def test_preflight_pass_all_green():
    checks = [Check("1 connectivity", "PASS", "ok"), Check("2 position", "PASS", "ok")]
    level, reason = assemble_preflight(checks)
    assert (level, reason) == ("PASS", "all checks green")


def test_preflight_any_fail_wins():
    checks = [Check("1 connectivity", "PASS", "ok"),
              Check("3 actuation", "FAIL", "belt off"),
              Check("2 position", "WARN", "end zone")]
    level, reason = assemble_preflight(checks)
    assert level == "FAIL"
    assert "belt off" in reason and "\n" not in reason


def test_preflight_pass_with_warn_and_skip():
    checks = [Check("1 connectivity", "WARN", "REBOOT detected"),
              Check("4 scope", "SKIP", "no --scope")]
    level, reason = assemble_preflight(checks)
    assert level == "PASS"
    assert "REBOOT" in reason and "skipped" in reason
