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

// ---------------------------------------------------------------------------
// L6 Ident (kueche-mief-spec §3, rows 1–32). Every constant cites its §3 row
// or, where §3 is silent, the reason (deviations: docs/findings/
// ident-sil-2026-11-07.md §Abweichungen). Times in ms, positions in %.
// ---------------------------------------------------------------------------

inline constexpr int IDENT_PROBE_WIDTHS = 4;
inline constexpr int IDENT_POSITIONS = 7;
inline constexpr int IDENT_REPS = 2;
inline constexpr int IDENT_IMP_WIDTHS = 6;
inline constexpr int IDENT_PHASES = 10;  // I0..I9
inline constexpr int IDENT_ESCAPE_WIDTHS = 5;

struct IdentConfig {
  // --- identity + priors supplied by the wrapper (YAML), never learned ---
  uint16_t fw_major = 1;        // stamp (§5): invalid on major mismatch
  uint8_t instance = 0;         // stamp (§5): R1 instance index
  // prior pot scale = the legacy calibrate_linear endpoints. Its ORIENTATION
  // is semantics (which physical end the user calls 0 %), the only pot fact
  // the device cannot learn; the magnitudes are only a first estimate for
  // I2/row 6 and are replaced by the I3 rail medians.
  float raw_hint_at_0 = 8000.0f;   // rig
  float raw_hint_at_100 = 100.0f;  // rig
  int8_t dir_sign_hint = 1;     // R6: invert_direction = I2 start value only
  float adc_full_scale = 8191.0f;  // 13-bit ADC1 (row 3 "an Schiene")
  float supply_lo = -1.0f;      // row 3 supply window [V]; lo >= hi = unchecked
  float supply_hi = -1.0f;

  // --- I1 PREFLIGHT (rows 3/4) ---
  int preflight_ms = 500;           // row 3 "σ-Probe 0,5 s"
  float adc_flat_max = 16.0f;       // counts: median at/below = "flach 0" (rig ADC
                                    // incident read exactly 0; real wiper >= ~100)
  float adc_rail_margin = 16.0f;    // counts below full scale = "an Schiene"
  float sigma_min_counts = 0.5f;    // MAD*1.48 below = stuck converter (rig sigma0 8.5)
  float sigma_max_counts = 150.0f;  // above = broken wiper (~2 % at 79 counts/%; rig x17)
  float thermal_preflight_max = 0.60f;  // row 3 (changed 2026-11-07): not tripped
                                        // and EMA < resume threshold

  // --- I2 POLARITY (rows 5–8) ---
  float probe_duty = 0.8f;          // row 5
  float probe_widths_ms[IDENT_PROBE_WIDTHS] = {5, 10, 20, 40};  // row 5
  int probe_rest_ms = 300;          // row 5
  float probe_motion_pct = 1.0f;    // row 5 (median of 8 samples)
  int probe_passes = 2;             // row 8 "Wiederholung x 2"
  // D23 rail start (Küche Slider 2, 2026-11-07): parked inside an ADC clamp
  // (preflight median at a clamp, sigma <= clip_sigma_max) the unit escapes
  // first: both signs alternately (hint direction "away" first), widths
  // escalating; success = the median leaves the clamp band AND moved >=
  // probe_motion_pct. The escape probe doubles as the polarity probe (row 7).
  float escape_widths_ms[IDENT_ESCAPE_WIDTHS] = {20, 40, 80, 150, 250};
  int escape_full_from = 3;         // widths from this index run at escape_duty_max
  float escape_duty_max = 1.0f;     // E3 / policy (a): full duty at a rail is ok
  float mid_lo = 15.0f;             // rows 5/6
  float mid_hi = 85.0f;

  // --- I3 ENDPOINTS (rows 9–12) ---
  float train_duty = 0.8f;          // row 9
  int train_width_ms = 20;          // row 9
  int train_period_ms = 150;        // row 9
  float train_min_progress_pct = 0.3f;  // row 9
  int train_stall_pulses = 3;       // row 9
  float rail_confirm_duty = 1.0f;   // deviation D3: a 0.8 stall is re-tried at
                                    // full duty before it counts as the rail
                                    // (high-breakaway units stop short at 0.8;
                                    // E3 / policy (a): 100 % at the rail is ok)
  int rail_rest_ms = 500;           // row 10
  int rail_median_ticks = 200;      // row 10
  float min_span_counts = 4000.0f;  // row 11
  float span_margin_pct = 4.0f;     // usable span [4, 96] % (= nominal dead band;
                                    // §3 gives no method for the dead band)
  // D22 (Küche Slider 2, 2026-11-07): a wiper that saturates the ADC at a rail
  // reads exactly the clamp value with sigma 0. Such a rail is VALID (sensor
  // dead band, controller-spec Gate answer 8) iff its median sits within
  // clip_tol of a clamp, sigma <= clip_sigma_max AND the train toward it saw
  // unclipped samples first (the wiper came from the live range — a flat-0 ADC
  // never does). raw_at := clamp value; the usable span ends span_margin_pct
  // inside the last unclipped reading of that train.
  float adc_clip_hi = 8191.0f;      // 13-bit full scale
  float adc_clip_lo = 0.0f;         // lower ADC clamp
  float clip_tol_counts = 16.0f;    // = adc_flat_max / adc_rail_margin
  float clip_sigma_max = 2.0f;      // counts: "sigma ~ 0" (< 1/4 of rig sigma0)

  // --- I4 QUIET (row 13) ---
  int quiet_ms = 2000;              // row 13
  int decimate = 4;                 // every 4th tick into the 512-sample MAD buffer

  // --- I5 BRACKET (rows 14–17) ---
  float bracket_start = 0.15f;      // row 14
  float bracket_step = 0.05f;       // row 14
  float bracket_fine = 0.03f;       // row 14 "Feinschritt 0,03 abwärts-bestätigt"
  float bracket_max = 0.95f;        // row 16
  int hold_ms = 300;                // row 14
  float bracket_motion_pct = 5.0f;  // row 14 (median of 8; principle 1)
  float quiet_band_pct = 1.0f;      // row 15 "letzter ruhiger Tick": within 1 %
  float bracket_adapt_margin = 0.10f;  // "adaptiv" (D5): later points start at
                                       // min(F_s so far) - 0.10 on the 0.05 grid,
                                       // never below bracket_start (120 s budget)
  bool fine_stop_at_quiet = false;  // PROPOSAL P1 (findings Nachtrag, OFF = spec
                                    // as built): end the downward fine walk once
                                    // the next try is at/below a coarse duty this
                                    // point already held quiet — that hold IS the
                                    // downward confirmation. Needs Jan's OK.
  float positions_pct[IDENT_POSITIONS] = {15, 26, 38, 50, 62, 74, 85};  // row 14
  int sustain_ms = 50;              // row 15 "Sustain-Bit nach 50 ms"
  float sustain_pct = 3.0f;         // auto-ident #5 "sustained >= 3 %"
  float fs_p95_lo = 0.25f;          // row 17
  float fs_p95_hi = 0.92f;          // row 17

  // --- R7 derivations (kueche-mief-spec §2 R7) ---
  float impulse_margin = 0.10f;     // R7a
  float impulse_lo = 0.60f;
  float impulse_hi = 0.95f;
  float umax_margin = 0.15f;        // R7b; lower clamp = impulse_duty (D7)
  float umax_hi = 1.0f;
  float vmin_margin = 1.15f;        // R7c: rig 120 %/s over the 104 %/s flight floor
  float vmin_lo = 80.0f;            // R7c floor: below the rig's measured 104 %/s

  // --- I6 STALL (row 18) ---
  int stall_ms = 2000;              // row 18
  float stall_floor = 0.10f;        // row 18
  float stall_below_fs = 0.10f;     // row 18
  float spike_sigma = 5.0f;         // row 18 ">5σ-Ticks"

  // --- I7 FLIGHTS (rows 19–21) ---
  float flight_d1 = 0.12f;          // row 19
  float flight_d2 = 0.20f;          // row 19
  float flight_max = 1.0f;          // D8: duties clamp at full scale ...
  float flight_min_gap = 0.06f;     // ... keeping a solvable 2x2 gap
  float flight_from = 25.0f;        // row 19
  float flight_to = 75.0f;
  int flight_slope_ms = 40;         // row 19 "letzte 40 ms"
  int flight_timeout_ms = 1500;     // a flight that never arrives -> v ~ 0 -> row 21
  int flight_reps = 2;              // row 20 "4 Flüge pro Richtung" = 2 duties x 2

  // --- I8 IMPULSES (rows 22–24) ---
  float imp_widths_ms[IDENT_IMP_WIDTHS] = {3, 5, 10, 20, 30, 50};  // row 22
  int imp_reps = 2;                 // row 22 ">= 2 Wdh."
  int imp_settle_ms = 300;          // post-pulse rest before the travel reading
                                    // (sim coast <= 60 ms; HIL creep, spec §6 d)
  float imp_start_fwd = 20.0f;      // D9: groups start mid-range and re-centre
  float imp_start_rev = 80.0f;      //     before a pulse could reach the guard
  float imp_guard_lo = 5.0f;
  float imp_guard_hi = 95.0f;
  float imp_predict = 1.5f;         // next travel ~ last * (w/w_last) * 1.5 + 1 %
  float imp_preload_pct = 4.0f;     // D9b: start approached from 4 % behind, so the
                                    // last repositioning pulse runs in the table
                                    // direction (takes up <= 2 % backlash, 2x tol)

  // --- repositioning (rows 14/22 "per Impulsen") ---
  float repo_tol_pct = 2.0f;        // bracket/flight start accuracy
  float repo_duty = 0.8f;           // = rows 5/9 duty ...
  float repo_escalate = 0.05f;      // ... +0.05 per zero-travel pulse, cap 1.0
  float repo_aim = 0.6f;            // aim 60 % of the gap (pulse travel superlinear)
  float repo_w_min_ms = 2.0f;       // = pulse_min_ms
  float repo_w_max_ms = 40.0f;      // <= row-5 probe maximum
  int repo_settle_ms = 150;         // = row 9 train period
  int pre_hold_settle_ms = 100;     // rest before a hold / flight reference (the
                                    // repositioning already waited repo_settle_ms;
                                    // this only refills the median-of-8)
  float repo_gain0 = 0.3f;          // %/ms first guess (rig: 6.2 % per 20 ms)

  // --- generic rows 27/29/30/32 ---
  float hand_pct = 3.0f;            // row 27
  int hand_win_ms = 100;            // row 27
  int hand_guard_ms = 60;           // coast exclusion after any pulse (sim coast
                                    // stops in <= 3 tau = 66 ms; rig creep 1.3 %
                                    // / 400 ms stays below 3 %)
  float supply_dev = 0.15f;         // row 30
  float adc_err_rate_max = 0.10f;   // row 32: EMA(100 ticks) of the error flag
  int adc_err_window_ticks = 100;   // row 32: EMA horizon of the error flag
  int hand_min_win_ms = 10;         // row 27: shortest usable comparison window
                                    // (>= one median-of-8 length + margin)
  float repo_gain_ema = 0.5f;       // repositioning: weight of the newest
                                    // travel/width sample (fast but damped)
  // row 29: expected phase time [s]; watchdog = 2 x expected + 1 s. Values are
  // worst cases over the SIL plant family (findings doc), not nominal times.
  float phase_expected_s[IDENT_PHASES] = {0.0f, 0.5f, 3.0f, 30.0f, 2.0f,
                                          100.0f, 2.0f, 30.0f, 40.0f, 0.0f};
};

}  // namespace slider_servo
