#pragma once

// slider_servo core — parameters (controller-spec.md §5: source of truth).
//
// Pure C++, zero IDF includes (spec §1 hard requirement). Every constant
// carries its basis; "tuned on the bench, forgot to write down" is forbidden
// (spec §4 #11). Per-unit calibration (UnitCalib) is what auto-ident fills
// and NVS stores; ServoConfig are compile-time defaults, HA-overridable
// where the §5 table says so.

#include <cstdint>

namespace slider_servo {

inline constexpr float FS_HZ = 1000.0f;   // loop rate (plan.md standing decision)
inline constexpr float DT = 1.0f / FS_HZ;

// ---------------------------------------------------------------------------
// Per-unit calibration: friction maps + impulse tables + sensor noise.
// Nominal values = docs/findings/plant-params.json snapshot 2026-07-17 (the
// same hermetic constants the Phase-3 prototype validated against). Every
// unit re-identifies these on device (auto-ident-spec.md); these defaults
// exist so a fresh unit is safe-but-suboptimal, never uncontrolled.
// ---------------------------------------------------------------------------

inline constexpr int KINETIC_NODES = 11;   // 0..100 % every 10 %
inline constexpr int STATIC_NODES = 21;    // 0..100 % every 5 %
inline constexpr int FWD_PULSE_POINTS = 8;
inline constexpr int REV_PULSE_POINTS = 6;

struct UnitCalib {
  // dynamics (duty units: force == stall-equivalent duty)
  float m = 1.22e-05f;   // duty/(%/s^2), sim-refined inertia (plant-model §3)
  float b = 5.47e-04f;   // duty/(%/s), viscous + brake damping (L9110 shorts at 0)

  // kinetic friction map F_k(p, dir), nodes every 10 %
  float fk_fwd[KINETIC_NODES] = {0.393f, 0.609f, 0.568f, 0.577f, 0.546f, 0.508f,
                                 0.576f, 0.539f, 0.528f, 0.532f, 0.580f};
  float fk_rev[KINETIC_NODES] = {0.501f, 0.623f, 0.554f, 0.586f, 0.548f, 0.560f,
                                 0.591f, 0.597f, 0.583f, 0.718f, 0.386f};

  // static (breakaway) map F_s(p, dir), nodes every 5 %
  float fs_fwd[STATIC_NODES] = {0.675f, 0.575f, 0.600f, 0.650f, 0.660f, 0.660f,
                                0.660f, 0.406f, 0.556f, 0.556f, 0.574f, 0.574f,
                                0.577f, 0.577f, 0.453f, 0.453f, 0.453f, 0.532f,
                                0.532f, 0.532f, 0.550f};
  float fs_rev[STATIC_NODES] = {0.695f, 0.695f, 0.675f, 0.675f, 0.500f, 0.500f,
                                0.500f, 0.629f, 0.602f, 0.602f, 0.611f, 0.700f,
                                0.700f, 0.700f, 0.720f, 0.720f, 0.720f, 0.720f,
                                0.720f, 0.720f, 0.700f};

  // measured M7 impulse tables at |duty| = 0.8 (plant-model §4c; the FF
  // primitive). Widths ms -> travel %. Rev column valid near mid-travel;
  // auto-ident #8 re-measures per unit at >=2 positions.
  float fwd_widths[FWD_PULSE_POINTS] = {2, 3, 5, 8, 10, 20, 30, 50};
  float fwd_travel[FWD_PULSE_POINTS] = {0.05f, 0.20f, 0.56f, 1.75f,
                                        2.64f, 6.21f, 10.26f, 21.66f};
  float rev_widths[REV_PULSE_POINTS] = {3, 5, 10, 20, 30, 50};
  float rev_travel[REV_PULSE_POINTS] = {0.28f, 0.62f, 1.02f,
                                        4.48f, 8.15f, 20.08f};

  // sensor noise (counts; sensorfit): gaussian floor, PWM-elevated
  float sigma0_counts = 8.5f;
  float sigma_pwm_counts = 11.3f;
  float counts_per_percent = 79.0f;  // |8000 - 100| / 100 (endpoint calib)

  // usable span: dead-band edges from endpoint calibration (Gate answer 8 —
  // targets clamp into this span in L4/L5; no L1 positional torque clamp)
  float span_lo = 4.0f;   // %
  float span_hi = 96.0f;  // %
};

// ---------------------------------------------------------------------------
// Compile-time controller defaults (spec §5 table; HA? column in the spec)
// ---------------------------------------------------------------------------

struct ServoConfig {
  // trajectory
  float v_cruise = 180.0f;    // %/s, validated flight regime
  float v_min_cmd = 120.0f;   // %/s, cliff floor + margin (flight from ~104 %/s)
  float a_traj = 5000.0f;     // %/s^2, ~1/3 of plant capability

  // cascade gains (SIL-tuned)
  float kp = 0.05f;           // duty per % reference error
  float kd = 2.5e-4f;         // duty per %/s velocity error
  float e_p_clamp = 3.0f;     // % reference-integrator freeze bound
  float u_max = 0.9f;         // above worst measured breakaway 0.80

  // approach / endgame
  float approach_margin = 1.5f;   // % below target the main move aims
  float endgame_zone = 3.5f;      // % : shorter moves are impulse-only
  float tol = 0.30f;              // % landing tolerance (UX bar is 2 %, Gate answer 6)
  float reapproach = 1.2f;        // % below target after an overshoot
  float impulse_duty = 0.8f;      // the duty the M7 table was measured at
  float impulse_aim = 0.90f;      // undershoot bias (fraction of gap)
  int impulse_settle_ms = 120;    // sim value — HIL re-tunes UP (post-pulse creep)
  int impulse_settle_short_ms = 70;  // pulses <= 12 ms
  int impulse_max = 8;
  int remove_max = 3;             // endgame re-approaches before gave_up
  float pulse_min_ms = 2.0f;      // identified table domain [2, 50] ms (T10)
  float pulse_max_ms = 50.0f;

  // stall escalation (progress-based, never v_hat — spec §2 row 6, T4)
  float stall_v = 25.0f;          // %/s progress threshold (above noise floor)
  int progress_win = 100;         // ticks
  int stall_ms = 80;
  float kick_boost = 0.08f;       // per escalation step; u_max in <=3 steps
  int kick_period_ms = 60;

  // grab (torque-saturation principle: u_max > worst breakaway discriminates)
  int grab_sat_ms = 120;
  int grab_pulses = 3;            // zero-travel pulses in WAIT = grab (row 12)
  float pulse_move_eps = 0.15f;   // % : below this a pulse counts as zero-travel
  int release_still_ms = 300;
  float still_v = 5.0f;           // %/s stillness bound in RELEASED
  float adopt_err = 1.5f;         // % hand-move adoption at rest (row 4)
  int adopt_ms = 120;

  // CRAWL (Gate answer 1; spec §2 rows 26–29)
  // crawl_impulse_duty: the duty of CRAWL pulse-train kicks. Defaults to
  // impulse_duty (0.8) so SIL/spec behaviour is unchanged; the wrapper may lower
  // it (e.g. 0.72, just above worst breakaway) to soften the acoustic clack —
  // gentler shove, less impact force. The closed crawl loop self-compensates the
  // pacing (each pulse then moves less than the 0.8-measured table predicts, so
  // it fires a touch more often); endgame pulses stay at impulse_duty.
  float crawl_impulse_duty = 0.8f;
  float v_crawl_max = 45.0f;      // %/s: 20 ms pulse (+6.2 %) per ~140 ms period;
                                  // (v_crawl_max, v_min_cmd) rounds up to v_min_cmd
  float crawl_min_step = 0.30f;   // % smallest commanded step (above p_hat noise)
  float crawl_max_width_ms = 20.0f;  // staircase-amplitude cap (smoothness)
  float crawl_fire_frac = 0.5f;   // fire when deficit >= this x expected travel
  float crawl_lead_frac = 1.5f;   // ratchet lead clamp, x expected travel
  int crawl_grab_pulses = 3;      // zero-travel pulses = grab (row 29)
  float crawl_override_err = 3.0f;  // % p_hat AHEAD of ratchet = hand drags (row 29)
  float crawl_degraded_frac = 0.5f; // achieved < this x requested over the window...
  int crawl_degraded_win = 2000;    // ...ticks -> crawl_degraded flag (honest, T8)

  // watchdog (row 19): 2 x expected move time + 1 s; expected includes endgame
  float watchdog_endgame_s = 1.5f;

  // L1 safety (basis: lab-rig interlocks, L9110 ~800 mA cont.; Gate answer 8:
  // thermal + budget are the ONLY current protection, no positional clamp)
  float thermal_tau_s = 10.0f;    // EMA time constant over |duty|
  float thermal_avg_max = 0.5f;   // trip level (lab-rig value)
  float thermal_resume = 0.35f;   // hysteresis: resume below this
  int ilk_yield_ms = 250;         // persistent clamp -> RELEASED (row 18)

  // observer (observer.py ObserverConfig; model_err 0.14 duty = SIL value)
  float obs_model_err_duty = 0.14f;
  float obs_err_corr_tau_s = 0.1f;
  float obs_gate_sigma = 5.0f;
  int obs_gate_max_consecutive = 8;
  float obs_v_stick = 3.0f;       // %/s stuck threshold in the process model
};

}  // namespace slider_servo
