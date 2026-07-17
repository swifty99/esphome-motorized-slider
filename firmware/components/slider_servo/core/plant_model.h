#pragma once

// Plant-model lookups shared by observer (L2) and controller (L3/L4):
// friction maps F_s/F_k and the impulse width<->travel tables. Pure functions
// over UnitCalib — no state, no allocation.

#include "servo_params.h"

namespace slider_servo {

// Linear interpolation over a fixed grid, clamped at the ends (numpy.interp
// semantics — the identified maps are defined on [0, 100] anyway).
inline float interp(float x, const float *xs, const float *ys, int n) {
  if (x <= xs[0]) return ys[0];
  if (x >= xs[n - 1]) return ys[n - 1];
  int i = 1;
  while (i < n - 1 && xs[i] < x) i++;
  const float t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
  return ys[i - 1] + t * (ys[i] - ys[i - 1]);
}

// Node grids are uniform by construction (every 10 % / 5 %).
inline float kinetic_node(int i) { return 10.0f * static_cast<float>(i); }
inline float static_node(int i) { return 5.0f * static_cast<float>(i); }

inline float fs_map(const UnitCalib &u, float p, float dir) {
  const float *f = dir > 0 ? u.fs_fwd : u.fs_rev;
  float nodes[STATIC_NODES];
  for (int i = 0; i < STATIC_NODES; i++) nodes[i] = static_node(i);
  return interp(p, nodes, f, STATIC_NODES);
}

inline float fk_map(const UnitCalib &u, float p, float dir) {
  const float *f = dir > 0 ? u.fk_fwd : u.fk_rev;
  float nodes[KINETIC_NODES];
  for (int i = 0; i < KINETIC_NODES; i++) nodes[i] = kinetic_node(i);
  return interp(p, nodes, f, KINETIC_NODES);
}

// Impulse table: expected travel [%] for a width [ms], and the inverse.
inline float pulse_travel(const UnitCalib &u, float width_ms, int dir) {
  if (dir > 0)
    return interp(width_ms, u.fwd_widths, u.fwd_travel, FWD_PULSE_POINTS);
  return interp(width_ms, u.rev_widths, u.rev_travel, REV_PULSE_POINTS);
}

inline float pulse_width_for(const UnitCalib &u, float travel_pct, int dir) {
  if (dir > 0)
    return interp(travel_pct, u.fwd_travel, u.fwd_widths, FWD_PULSE_POINTS);
  return interp(travel_pct, u.rev_travel, u.rev_widths, REV_PULSE_POINTS);
}

}  // namespace slider_servo
