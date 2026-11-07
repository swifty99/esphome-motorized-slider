#pragma once

// Seeded bugs for the SIL negative controls (plan.md Phase 5: every tripwire
// test must FAIL against its seeded bug, proving the test non-vacuous).
//
// The enum is always defined (it is only names); every check site goes
// through SIL_BUG(), which is the literal constant `false` unless the build
// defines SLIDER_SIL_BUGS — production firmware contains no live bug path.

namespace slider_servo {

enum class SeededBug : int {
  NONE = 0,
  T1_HOLD_TERM = 1,          // small P-hold term while IDLE
  T2_LOW_VMIN = 2,           // v_min_cmd lowered to 30 %/s (creep commands)
  T3_FF_TORQUE_SIGN = 3,     // stiction FF keyed to last torque sign (legacy bug)
  T4_STALL_FROM_VHAT = 4,    // stall detection reads v_hat instead of progress
  T5_NO_GATE_ESCAPE = 5,     // innovation gate without the consecutive-reject escape
  T6_NO_RELEASE = 6,         // grab declared but torque kept
  T7_REV_LANDING = 7,        // direct reverse landings allowed
  T8_CLEAR_GAVE_UP = 8,      // gave_up flag cleared before the settle latch
  T9_NO_WATCHDOG = 9,        // watchdog AND grab detection disabled
  T10_LONG_PULSE = 10,       // 200 ms "pulse" outside the table domain
  T11_KICK_UNCAPPED = 11,    // escalation kick ignores u_max
  T12_DUTY_IN_Z = 12,        // commanded duty blended into the measurement
  T13_RECORDER_STALL = 13,   // full recorder stalls the control pipeline
  T14_CLAMP_LATE = 14,       // L1 clamp applied one tick late
  T15_ROGUE_TRANSITION = 15, // undocumented transition BRAKE -> MOVE on retarget
  T16_CRAWL_HOLD = 16,       // inter-pulse hold duty during CRAWL
  T17_NO_DRAG_CHECK = 17,    // CRAWL ratchet-error override check dropped
  G1_NO_KEEPROLL = 18,       // GLIDE uses a long settle -> re-sticks between kicks
  G2_GLIDE_HOLD = 19,        // inter-kick hold duty during GLIDE (breaks T16/G-T2)
  // kueche-mief-spec §4 (Ident L6 / R7 / R8)
  K2_BRACKET_RIG = 20,       // bracket starts at 0.60, I6 stall fixed 0.45
  K3_DIR_FIXED = 21,         // dir_sign fixed +1 (polarity never learned)
  K4_EARLY_COMMIT = 22,      // result published (NVS) before I9
  K5_NO_HAND = 23,           // row 27 (hand at duty 0) deleted
  K7_RIG_DUTY = 24,          // R7 derivation replaced by fixed 0.8 / 0.9
  K8_UNID_FLAG_CLEAR = 25,   // "nicht identifiziert" flag cleared after first move
  K9_SINGLE_PROBE = 26,      // averaging replaced by a single probe
  KF_ROGUE_IDENT = 27,       // undocumented transition: CMD Ident in I5 restarts I1
  KC_CLIP_REJECT = 28,       // ADC-clipped rail rejected by the sigma window (pre-fix)
  KR_RAILSTART_OFF = 29,     // start parked in an ADC clamp -> old preflight FAIL
};

#ifdef SLIDER_SIL_BUGS
#define SIL_BUG(member, which) ((member) == ::slider_servo::SeededBug::which)
#else
#define SIL_BUG(member, which) (false)
#endif

}  // namespace slider_servo
