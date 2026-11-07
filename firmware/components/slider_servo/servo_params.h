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

  // --- kueche-mief-spec §5 / R6–R8 (filled by Ident L6; defaults = rig) ---
  // R6: duty sign -> position direction. +1: positive motor duty raises the
  // calibrated position. Servo::tick_raw() multiplies the core's output by it;
  // Servo::tick(z_pct) (legacy % path) never reads it.
  int8_t dir_sign = 1;
  // endpoint calibration in raw ADC counts (rig: 8000 @ 0 %, 100 @ 100 %, the
  // SensorSim / plant-model constants). Used by Servo::tick_raw() only.
  float raw_at_0 = 8000.0f;
  float raw_at_100 = 100.0f;
  // R7 per-unit derivations. Read by the controller ONLY when derived == true;
  // otherwise ServoConfig's impulse_duty / u_max / v_min_cmd apply (the old
  // constants — SIL byte-identical, pattern of commit e28f965).
  bool derived = false;
  float impulse_duty = 0.8f;   // R7a clamp(F_s_p95 + 0.10, 0.60, 0.95)
  float u_max = 0.9f;          // R7b clamp(F_s_p95 + 0.15, impulse_duty, 1.0)
  float v_min_cmd = 120.0f;    // R7c from the measured flight law (Ident I7)
  // R8: false = no valid NVS ident -> throttled mode (impulse path only, no
  // MOVE cruise, small widths, status flag every tick). Default false because
  // a fresh unit IS unidentified; the SIL runner sets it explicitly to true
  // for the legacy suite (the nominal map is the rig's identified snapshot).
  bool identified = false;
  // Ident by-products the core does not consume (stored for the report /
  // plantreport cross-check, §5): per-direction kinetic level + damping.
  float fk_mid_fwd = 0.55f;
  float fk_mid_rev = 0.55f;
  float b_fwd = 5.47e-04f;
  float b_rev = 5.47e-04f;
  float spike_rate = 0.0f;     // fraction of PWM ticks beyond 5 sigma0 (I6)
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

  // self-adjusting CRAWL step (OPTIONAL; default disabled = legacy dosing).
  // When crawl_step > 0 each crawl pulse aims a constant small travel and the
  // pulse width self-corrects online from the actually-measured travel of the
  // previous pulse — regular quiet steps, robust to per-unit / per-direction
  // impulse-table error. Every default below preserves today's behaviour.
  float crawl_step = 0.0f;        // %; target per-pulse travel in self-adjusting
                                  // CRAWL. 0 = disabled (use the legacy
                                  // deficit-dosed width). Basis: acoustic — a
                                  // small constant step gives a regular quiet
                                  // cadence; HIL-tuned.
  float crawl_w_gain = 0.30f;     // dimensionless EMA gain for the width-scale
                                  // update per pulse. Basis: slow closed-loop
                                  // correction of impulse-table error.
  float crawl_w_scale_min = 0.5f; // bounds on the adaptive width scale. Basis:
  float crawl_w_scale_max = 2.0f; // keep dosing inside the table's validated
                                  // ±factor.
  int crawl_settle_ms = 0;        // 0 = use the endgame short/long settle
                                  // (byte-identical default). >0 = crawl-
                                  // EXCLUSIVE inter-pulse settle (ms): the
                                  // period lever — a shorter settle tightens the
                                  // clack cadence and shrinks the per-pulse step
                                  // at a given rate, WITHOUT touching the endgame
                                  // settle A1 depends on. HIL-tuned.
  float crawl_land_zone = 0.0f;   // 0 = hand off to the endgame at endgame_zone
                                  // (byte-identical default). >0 = crawl carries
                                  // its gentle steps to within this %-of-target
                                  // before the endgame takes over — kills the
                                  // settling judder (the endgame otherwise closes
                                  // the whole endgame_zone with big full-duty
                                  // lunges after the small crawl steps). Crawl-
                                  // only; the fast-move endgame is untouched (A1).

  // GLIDE (glide-mode-spec.md; gate answers 2026-07-17): "go there as slow as
  // possible but NEVER STOP, nail it first shot, self-learning." A continuous
  // keep-rolling kick train — re-kick BEFORE the slider re-sticks so motion
  // never stalls and the belt never unloads. OPTIONAL; default disabled
  // (glide == 0) => slow moves use CRAWL and SIL stays byte-identical. When
  // glide > 0 a slow request (0 < v_req <= v_crawl_max) enters State::GLIDE.
  // Reuses the CRAWL ratchet / self-adjust EMA / grab machinery.
  float glide = 0.0f;             // 0 = off (CRAWL). >0 = GLIDE for slow moves.
  float glide_floor = 40.0f;      // %/s never-stop floor: the requested rate is
                                  // clamped UP to this so the kick cadence stays
                                  // tight enough that the slider never re-sticks.
                                  // On THIS plant (m/fk re-stick clock, spec §1)
                                  // you cannot go slower AND never stop; not-
                                  // stopping is the priority (gate Q1). HIL-tuned.
  float glide_step = 0.4f;        // %; target per-kick travel (small = quiet,
                                  // tight cadence). Reuses the crawl self-adjust
                                  // width EMA (crawl_w_gain / crawl_w_scale_*).
  int glide_settle_ms = 3;        // ms min forced gap after a kick — short so the
                                  // next kick lands before re-stick (keep-rolling,
                                  // vs the crawl 25–70 ms that lets it re-stick).
  float v_roll = 30.0f;           // %/s short-window p̂-progress floor: at/below
                                  // this the slider is about to re-stick -> fire
                                  // early to keep rolling. Just above the 25 %/s
                                  // progress noise floor (never v_hat, spec §1c).
  int glide_prog_win = 12;        // ticks; short p̂-progress window for the keep-
                                  // rolling detector. Short = fast but noisier.
  float glide_runaway_v = 200.0f; // %/s: a kick over-delivered into a mini-flight
                                  // -> skip the next kick, coast down (bounded).
  int glide_max_gap_ms = 20;      // hard inter-kick cap (safety; ~2x worst re-
                                  // stick time). A last-resort kick so it never
                                  // stops. Kept ABOVE the rate cadence so it does
                                  // not bind normally (a binding cap would push
                                  // the plant ahead of the ratchet and read as a
                                  // hand-drag). ratchet/rate_due drive the cadence.
  float coast_zone = 0.8f;        // % remaining at which GLIDE hands to the gentle
                                  // single forward seat (nail first shot, no
                                  // judder — gate Q3). 0 => crawl_land_zone/
                                  // endgame_zone. Stage 2 adds the learned coast.

  // R8 unidentified mode (kueche-mief-spec §2 R8: "nur Endgame-/Impuls-Pfad mit
  // kleinen Breiten, kein MOVE-Cruise"). Active iff UnitCalib::identified ==
  // false. Every move — any distance, any speed request — runs the endgame
  // pulse path (rows 2/14/15) with widths capped; rows 3/13/26 (MOVE / CRAWL /
  // GLIDE entry) are suppressed.
  float unid_max_width_ms = 10.0f;  // "kleine Breiten": nominal table 10 ms =
                                    // 2.6 % (< endgame_zone 3.5 %); on a
                                    // breakaway-0.43 unit ~3x that, still a
                                    // bounded hop the endgame re-measures
  int unid_impulse_max = 60;        // pulse budget (row 11) in unidentified mode:
                                    // a 0->100 % move at ~2 % per capped pulse
                                    // needs ~50; bounded so T9 still holds
  float unid_rate = 8.0f;           // %/s watchdog pacing (row 19) for the pulse
                                    // path: 10 ms pulse 2.6 % per 130 ms ~ 20 %/s
                                    // nominal, /2.5 margin for slower units

  // watchdog (row 19): 2 x expected move time + 1 s; expected includes endgame
  float watchdog_endgame_s = 1.5f;

  // L1 safety (Gate answer 8: thermal + budget are the ONLY current protection,
  // no positional clamp). Source: Jan 2026-11-07, "5 min Vollgas unkritisch" —
  // the old lab-rig values (tau 10 s, trip 0.5, resume 0.35) tripped after ~7 s
  // of full duty, which is unrealistically sharp. New: at |duty| = 1.0 the EMA
  // reaches the trip after -tau*ln(1 - 0.95) = 120 * 3.0 ~= 360 s (~6 min),
  // i.e. 5 min of uninterrupted full duty never trips; resume after cooling
  // below 0.60. Tripwire: test_thermal.py.
  float thermal_tau_s = 120.0f;   // EMA time constant over |duty| (Jan 2026-11-07, 5 min Vollgas unkritisch)
  float thermal_avg_max = 0.95f;  // trip level (Jan 2026-11-07, 5 min Vollgas unkritisch)
  float thermal_resume = 0.60f;   // hysteresis: resume below this (Jan 2026-11-07, 5 min Vollgas unkritisch)
  int ilk_yield_ms = 250;         // persistent clamp -> RELEASED (row 18)

  // observer (observer.py ObserverConfig; model_err 0.14 duty = SIL value)
  float obs_model_err_duty = 0.14f;
  float obs_err_corr_tau_s = 0.1f;
  float obs_gate_sigma = 5.0f;
  int obs_gate_max_consecutive = 8;
  float obs_v_stick = 3.0f;       // %/s stuck threshold in the process model
};

}  // namespace slider_servo
