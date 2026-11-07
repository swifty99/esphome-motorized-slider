// sim_runner --ident: plays an ident scenario against a TRUTH plant from the
// kueche-mief-spec §4 plant family. The servo core sees raw ADC counts only
// (RawSensor) and drives MOTOR duty (tick_raw); the truth plant, its
// polarity, orientation, friction and supply are invisible to it. After the
// ident the same core keeps running — on the identified UnitCalib — so moves
// after "DONE" test the result, not the nominal map.
//
// Scenario directives (one per line, '#' comments):
//   p0 <x>                 true start position (plant coordinate x, 0..100)
//   seed <n> | duration <s> | perturb <d_fwd> <d_rev> <d_m> <d_b>
//   friction_scale <k>     fs AND fk x k (breakaway family)
//   motor_pol <+1|-1|0>    pot_inv <0|1>    pot_raw <lo> <hi>
//   noise <k> | backlash <pct> | supply_scale <s> | fs_spread <rel>
//   supply_adc <V_nom>     supply ADC present, reads V_nom * supply_scale
//   supply_window <lo> <hi> | hint <raw0> <raw100> | dir_hint <+1|-1>
//   fs_patch <lo> <hi> <fs> breakaway := fs on plant nodes in [lo, hi] %
//   boot_nvs               adopt the synthetic previous NVS result at boot
//   csv_every <n>          CSV decimation (default 1)
//   at <t> <event...>
//   on <PHASE>[#n] <delay> <event...>   PHASE in I1..I9 DONE FAILED ABORTED END
// Events: ident | abort_ident | target <pct> [speed <v>] | release |
//   abort on|off | grab <s> | drag_rel <dx> <s> | step <dx> | supply <s> |
//   friction <k> | adc flat|rail|ok|stuck <raw> | nan

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../firmware/components/slider_servo/servo.h"
#include "plant_sim.h"

using namespace slider_servo;
using namespace slider_sil;

namespace {

struct IEvent {
  int tick = -1;            // absolute (at) or -1 until triggered (on)
  std::string trigger;      // "" for at-events
  int occurrence = 1;
  int delay = 0;
  bool fired = false;
  std::vector<std::string> args;
};

struct IScenario {
  float p0 = 50.0f;
  uint32_t seed = 1;
  float duration_s = 150.0f;
  PlantPerturb perturb;
  PlantVariant var;
  float supply_nom = -1.0f;
  float win_lo = -1.0f, win_hi = -1.0f;
  float hint0 = 8000.0f, hint100 = 100.0f;
  int dir_hint = 1;
  bool boot_nvs = false;
  int csv_every = 1;
  float patch_lo = -1.0f, patch_hi = -1.0f, patch_fs = 0.0f;  // tight spot
  std::vector<IEvent> events;
};

bool parse(const char *path, IScenario &sc) {
  std::ifstream f(path);
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    const auto h = line.find('#');
    if (h != std::string::npos && line.compare(0, 3, "on ") != 0) line = line.substr(0, h);
    else if (h != std::string::npos) {
      // "on DONE#2 ..." uses '#': strip comments only after a " #"
      const auto c = line.find(" #");
      if (c != std::string::npos) line = line.substr(0, c);
    }
    std::istringstream ss(line);
    std::string tok;
    if (!(ss >> tok)) continue;
    if (tok == "p0") ss >> sc.p0;
    else if (tok == "seed") ss >> sc.seed;
    else if (tok == "duration") ss >> sc.duration_s;
    else if (tok == "perturb")
      ss >> sc.perturb.d_fwd >> sc.perturb.d_rev >> sc.perturb.d_m >> sc.perturb.d_b;
    else if (tok == "friction_scale") ss >> sc.var.friction_scale;
    else if (tok == "motor_pol") ss >> sc.var.motor_pol;
    else if (tok == "pot_inv") { int v; ss >> v; sc.var.pot_inv = v != 0; }
    else if (tok == "pot_raw") ss >> sc.var.raw_lo >> sc.var.raw_hi;
    else if (tok == "noise") ss >> sc.var.noise_scale;
    else if (tok == "backlash") ss >> sc.var.backlash;
    else if (tok == "supply_scale") ss >> sc.var.supply_scale;
    else if (tok == "fs_spread") ss >> sc.var.fs_spread;
    else if (tok == "supply_adc") ss >> sc.supply_nom;
    else if (tok == "supply_window") ss >> sc.win_lo >> sc.win_hi;
    else if (tok == "hint") ss >> sc.hint0 >> sc.hint100;
    else if (tok == "dir_hint") ss >> sc.dir_hint;
    else if (tok == "boot_nvs") sc.boot_nvs = true;
    else if (tok == "csv_every") ss >> sc.csv_every;
    else if (tok == "fs_patch") ss >> sc.patch_lo >> sc.patch_hi >> sc.patch_fs;
    else if (tok == "at" || tok == "on") {
      IEvent e;
      if (tok == "at") {
        float t;
        ss >> t;
        e.tick = static_cast<int>(t * 1000.0f + 0.5f);
      } else {
        std::string trig;
        float d;
        ss >> trig >> d;
        const auto hp = trig.find('#');
        if (hp != std::string::npos) {
          e.occurrence = std::atoi(trig.c_str() + hp + 1);
          trig = trig.substr(0, hp);
        }
        e.trigger = trig;
        e.delay = static_cast<int>(d * 1000.0f + 0.5f);
      }
      std::string a;
      while (ss >> a) e.args.push_back(a);
      if (e.args.empty()) return false;
      sc.events.push_back(e);
    } else {
      std::fprintf(stderr, "ident scenario: unknown directive '%s'\n", tok.c_str());
      return false;
    }
  }
  return true;
}

uint32_t fnv(const void *p, size_t n) {
  const uint8_t *b = static_cast<const uint8_t *>(p);
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < n; i++) {
    h ^= b[i];
    h *= 16777619u;
  }
  return h;
}

const char *STATE_NAMES[] = {"IDLE", "MOVE", "BRAKE", "PULSE", "WAIT",
                             "CRAWL", "RELEASED", "GLIDE", "IDENT"};
const char *FLAG_NAMES[] = {"none", "reached", "gave_up", "adopted"};
const char *REASON_NAMES[] = {"none", "grab", "interlock", "watchdog", "fault", "command"};
const char *OUTCOME_NAMES[] = {"none", "ok", "failed", "aborted", "rejected"};

// travel of one isolated pulse on a fresh copy of the truth plant (frame %)
float truth_pulse(const TruthPlant &tp, float gain_abs, float x0, float pdir,
                  float duty, int width) {
  PlantStepper ps;
  ps.init(&tp, x0);
  for (int i = 0; i < width; i++) ps.step(pdir * duty * gain_abs);
  for (int i = 0; i < 400; i++) ps.step(0.0f);
  return (ps.p() - x0) * pdir;
}

struct Run {
  int start = -1, end = -1;
  int enter[10] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
  IdentResult res;
  int outcome = 0, reason = 0;
  float post_u_max = 0.0f;
  bool post_open = false;
};

struct Move {
  float target = 0.0f;
  int issued = -1, settled = -1;
  float final_frame = -1.0f;
  float pulse_u_max = 0.0f;
  float u_max = 0.0f;
  int move_ticks = 0;
  int status = 0;
  std::string flag = "none";
  bool active = false;
  bool left_idle = false;
};

}  // namespace

int run_ident_mode(const char *path, const char *out_path, int bug) {
  IScenario sc;
  if (!parse(path, sc)) {
    std::fprintf(stderr, "cannot parse ident scenario %s\n", path);
    return 2;
  }
  UnitCalib nominal;          // prior: the nominal map, NOT identified (R8)
  nominal.identified = false;
  ServoConfig cfg;
  IdentConfig icfg;
  icfg.raw_hint_at_0 = sc.hint0;
  icfg.raw_hint_at_100 = sc.hint100;
  icfg.dir_sign_hint = static_cast<int8_t>(sc.dir_hint >= 0 ? 1 : -1);
  icfg.supply_lo = sc.win_lo;
  icfg.supply_hi = sc.win_hi;

  TruthPlant base = TruthPlant::from(nominal, sc.perturb);
  if (sc.patch_hi > sc.patch_lo)  // a tight spot on the track (both directions)
    for (int k = 0; k < STATIC_NODES; k++) {
      const float x = 5.0f * static_cast<float>(k);
      if (x >= sc.patch_lo && x <= sc.patch_hi) base.u.fs_fwd[k] = base.u.fs_rev[k] = sc.patch_fs;
    }
  TruthPlant truth = base;
  scale_friction(truth, base, sc.var.friction_scale);
  float supply_scale = sc.var.supply_scale;
  PlantStepper plant;
  plant.init(&truth, sc.p0);
  plant.set_drive_gain(static_cast<float>(sc.var.motor_pol) * supply_scale);
  if (sc.var.fs_spread > 0.0f) plant.set_fs_spread(sc.var.fs_spread, sc.seed * 7919u + 13u);
  RawSensor sensor;
  sensor.init(&nominal, sc.seed, sc.var, sc.p0);

  // frame: 0 % = the rail whose raw is on the hint-0 side (semantics, §3 row 11)
  const float r_x0 = sensor.raw_at_x(0.0f), r_x100 = sensor.raw_at_x(100.0f);
  const bool frame_same = ((sc.hint100 - sc.hint0) > 0) == ((r_x100 - r_x0) > 0);
  auto frame = [&](float x) { return frame_same ? x : 100.0f - x; };

  // synthetic previous NVS stamp (K4 must find it bit-identical after a FAIL)
  IdentResult nvs;
  std::memset(static_cast<void *>(&nvs), 0, sizeof(nvs));
  {
    IdentResult prev;
    prev.valid = true;
    prev.calib = nominal;
    prev.calib.identified = true;
    prev.stamp.fw_major = icfg.fw_major;
    prev.stamp.instance = icfg.instance;
    prev.stamp.endpoint_id = ident_endpoint_id(prev.calib);
    prev.stamp.commit_counter = 7;
    prev.stamp.date_counter = 20251;
    std::memcpy(static_cast<void *>(&nvs), &prev, sizeof(nvs));
  }
  const uint32_t nvs_hash0 = fnv(&nvs, sizeof(nvs));

  Servo servo;
  const float raw_first = sensor.raw_at_x(sc.p0);
  const float p_init = (raw_first - nominal.raw_at_0) / (nominal.raw_at_100 - nominal.raw_at_0) * 100.0f;
  servo.init(&nominal, &cfg, p_init, 0, static_cast<SeededBug>(bug), &icfg);
  if (sc.boot_nvs) servo.adopt_ident(nvs, raw_first);
  uint32_t seen_seq = servo.ident_commit_seq();
  int nvs_writes = 0;

  std::FILE *csv = nullptr;
  if (out_path) {
    csv = std::fopen(out_path, "w");
    if (!csv) return 2;
    std::fprintf(csv, "tick,pos,pos_frame,raw,u,u_cmd,state,ident,p_hat,unid,flag,reason\n");
  }

  std::vector<Run> runs;
  std::vector<Move> moves;
  int occ[16] = {0};  // per-trigger occurrence counters
  auto trig_id = [](const std::string &t) -> int {
    if (t.size() == 2 && t[0] == 'I') return t[1] - '0';
    if (t == "DONE") return 10;
    if (t == "FAILED") return 11;
    if (t == "ABORTED") return 12;
    if (t == "END") return 13;
    return -1;
  };
  auto trigger = [&](int id, int now) {
    if (id < 0) return;
    occ[id]++;
    for (auto &e : sc.events)
      if (!e.trigger.empty() && e.tick < 0 && trig_id(e.trigger) == id &&
          e.occurrence == occ[id])
        e.tick = now + e.delay;
  };

  int grab_until = -1;
  float grab_pos = 0.0f;
  int drag_until = -1, drag_start = 0;
  float drag_from = 0.0f, drag_to = 0.0f;
  bool nan_next = false;
  float u_motor = 0.0f;
  IdentState prev_is = servo.ident_state();
  State prev_cs = servo.controller().state();
  int grab_detect_post = -1;
  int ident_done_tick = -1;
  const int n_max = static_cast<int>(sc.duration_s * 1000.0f);
  // fuzz bookkeeping (T15 pattern for L6): observed ident transitions, and
  // the exclusivity invariants while a run is active
  int trans[12][12] = {};
  int ctrl_violation = 0, rej_during = 0, acc_during = 0, starts_during = 0;

  for (int i = 0; i < n_max; i++) {
    for (auto &e : sc.events) {
      if (e.fired || e.tick < 0 || e.tick > i) continue;
      e.fired = true;
      const std::string &k = e.args[0];
      auto arg = [&](size_t j) { return j < e.args.size() ? std::atof(e.args[j].c_str()) : 0.0; };
      if (k == "ident") {
        if (servo.ident_active()) starts_during++;
        servo.start_ident(20260 + static_cast<uint32_t>(runs.size()), true);
      } else if (k == "abort_ident") {
        servo.abort_ident();
      } else if (k == "target") {
        Move m;
        m.target = static_cast<float>(arg(1));
        const float v = (e.args.size() > 3 && e.args[2] == "speed") ? static_cast<float>(arg(3)) : 0.0f;
        const bool during = servo.ident_active();
        m.status = static_cast<int>(servo.set_target(m.target, v));
        if (during) (m.status != 0 ? rej_during : acc_during)++;
        m.issued = i;
        m.active = true;
        for (auto &o : moves) o.active = false;
        moves.push_back(m);
      } else if (k == "release") {
        servo.release();
      } else if (k == "abort") {
        servo.set_abort(e.args.size() > 1 && e.args[1] == "on");
      } else if (k == "grab") {
        grab_until = i + static_cast<int>(arg(1) * 1000.0);
        grab_pos = plant.p();
      } else if (k == "drag_rel") {
        drag_start = i;
        drag_until = i + static_cast<int>(arg(2) * 1000.0);
        drag_from = plant.p();
        drag_to = plant.p() + static_cast<float>(arg(1));
      } else if (k == "step") {
        plant.force(plant.p() + static_cast<float>(arg(1)), 0.0f, true);
      } else if (k == "supply") {
        supply_scale = static_cast<float>(arg(1));
        plant.set_drive_gain(static_cast<float>(sc.var.motor_pol) * supply_scale);
      } else if (k == "friction") {
        scale_friction(truth, base, sc.var.friction_scale * static_cast<float>(arg(1)));
      } else if (k == "adc") {
        const std::string m = e.args.size() > 1 ? e.args[1] : "ok";
        if (m == "flat") sensor.set_fault(RawSensor::Fault::FLAT);
        else if (m == "rail") sensor.set_fault(RawSensor::Fault::RAIL);
        else if (m == "stuck") sensor.set_fault(RawSensor::Fault::STUCK, static_cast<float>(arg(2)));
        else sensor.set_fault(RawSensor::Fault::NONE);
      } else if (k == "heat") {
        servo.preload_thermal(static_cast<float>(arg(1)));
      } else if (k == "nan") {
        nan_next = true;
      }
    }

    {  // commands act between ticks: observe their transition too (abort -> ABORT)
      const IdentState is0 = servo.ident_state();
      if (is0 == IdentState::ABORT && is0 != prev_is) {
        trans[static_cast<int>(prev_is)][static_cast<int>(is0)]++;
        prev_is = is0;
      }
    }
    const float supply_v = sc.supply_nom > 0.0f ? sc.supply_nom * supply_scale : -1.0f;
    float raw = sensor.sample_raw(plant.p(), u_motor);
    if (nan_next) {
      raw = std::nanf("");
      nan_next = false;
    }
    u_motor = servo.tick_raw(raw, supply_v, false);
    plant.step(u_motor);

    if (grab_until >= 0 && i < grab_until) {
      plant.force(grab_pos, 0.0f, true);
    } else if (drag_until >= 0 && i < drag_until) {
      const float frac = static_cast<float>(i - drag_start) /
                         static_cast<float>(drag_until - drag_start);
      const float vel = (drag_to - drag_from) /
                        (static_cast<float>(drag_until - drag_start) * 0.001f);
      plant.force(drag_from + (drag_to - drag_from) * frac, vel, false);
    } else if (drag_until >= 0 && i == drag_until) {
      plant.force(plant.p(), 0.0f, true);
      drag_until = -1;
    }

    // ident bookkeeping
    const IdentState is = servo.ident_state();
    if (servo.ident_active() && servo.controller().state() != State::IDENT) ctrl_violation++;
    if (is != prev_is) {
      trans[static_cast<int>(prev_is)][static_cast<int>(is)]++;
      if (prev_is == IdentState::I0_OFF && servo.ident_active()) {
        Run r;
        r.start = i;
        runs.push_back(r);
        for (auto &o : runs) o.post_open = false;
      }
      const int id = static_cast<int>(is);
      if (id >= 1 && id <= 9) {
        trigger(id, i);
        if (!runs.empty()) runs.back().enter[id] = i;
      }
      if (is == IdentState::I0_OFF && !runs.empty()) {
        Run &r = runs.back();
        r.end = i;
        r.post_open = true;
        r.res = servo.ident().work();
        r.outcome = static_cast<int>(servo.ident_outcome());
        r.reason = static_cast<int>(servo.ident_fail_reason());
        const IdentOutcome oc = servo.ident_outcome();
        if (oc == IdentOutcome::OK) {
          trigger(10, i);
          if (ident_done_tick < 0) ident_done_tick = i;
        } else if (oc == IdentOutcome::FAILED) trigger(11, i);
        else if (oc == IdentOutcome::ABORTED) trigger(12, i);
        trigger(13, i);
      }
      prev_is = is;
    }
    for (auto &r : runs)
      if (r.post_open && std::fabs(u_motor) > r.post_u_max) r.post_u_max = std::fabs(u_motor);
    if (servo.ident_commit_seq() != seen_seq) {  // the wrapper's NVS write
      seen_seq = servo.ident_commit_seq();
      std::memcpy(static_cast<void *>(&nvs), &servo.ident_result(), sizeof(nvs));
      nvs_writes++;
    }

    const Controller &c = servo.controller();
    if (!moves.empty() && moves.back().active) {
      Move &m = moves.back();
      // a move ends when the controller reaches IDLE again (or RELEASED->IDLE)
      if (c.state() != State::IDLE) m.left_idle = true;
      if (c.state() == State::MOVE) m.move_ticks++;
      if (std::fabs(u_motor) > m.u_max) m.u_max = std::fabs(u_motor);
      if (c.state() == State::PULSE && std::fabs(u_motor) > m.pulse_u_max)
        m.pulse_u_max = std::fabs(u_motor);
      if (m.left_idle && c.state() == State::IDLE && m.settled < 0) {
        m.settled = i;
        m.final_frame = frame(plant.p());
        m.flag = FLAG_NAMES[static_cast<int>(c.settle_flag())];
      }
    }
    if (ident_done_tick >= 0 && grab_detect_post < 0 && c.state() == State::RELEASED &&
        prev_cs != State::RELEASED && c.release_reason() == ReleaseReason::GRAB)
      grab_detect_post = i;
    prev_cs = c.state();

    if (csv && (i % sc.csv_every) == 0) {
      std::fprintf(csv, "%d,%.4f,%.4f,%.2f,%.4f,%.4f,%s,%d,%.4f,%d,%s,%s\n", i, plant.p(),
                   frame(plant.p()), raw, u_motor, servo.u_cmd_last(),
                   STATE_NAMES[static_cast<int>(c.state())], static_cast<int>(is),
                   servo.observer().p_hat(), c.unidentified_flag() ? 1 : 0,
                   FLAG_NAMES[static_cast<int>(c.settle_flag())],
                   REASON_NAMES[static_cast<int>(c.release_reason())]);
    }
  }
  if (csv) std::fclose(csv);

  // ------------------------------------------------------------- summary --
  const float gain_abs = std::fabs(supply_scale);
  std::printf("frame_same=%d\n", frame_same ? 1 : 0);
  std::printf("truth_dir_sign=%d\n", sc.var.motor_pol * (frame_same ? 1 : -1));
  {
    // truth breakaway at the ident positions in COMMANDED duty (÷ supply)
    float all[2 * IDENT_POSITIONS];
    int n = 0;
    for (int dir = 0; dir < 2; dir++)
      for (int k = 0; k < IDENT_POSITIONS; k++) {
        const float P = icfg.positions_pct[k];
        const float D = dir == 0 ? 1.0f : -1.0f;
        const float x = frame_same ? P : 100.0f - P;
        const float pd = frame_same ? D : -D;
        const float v = fs_map(truth.u, x, pd) / gain_abs;
        all[n++] = v;
        std::printf("truth_fs_%s_%d=%.4f\n", dir == 0 ? "fwd" : "rev", k, v);
      }
    std::sort(all, all + n);
    const float pos = 0.95f * static_cast<float>(n - 1);
    const int ii = static_cast<int>(pos);
    std::printf("truth_fs_p95=%.4f\ntruth_fs_min=%.4f\n",
                all[ii] + (pos - ii) * (all[ii + 1] - all[ii]), all[0]);
    for (int dir = 0; dir < 2; dir++) {
      float s = 0.0f;
      int m = 0;
      for (float P = 25.0f; P <= 75.0f; P += 5.0f, m++) {
        const float D = dir == 0 ? 1.0f : -1.0f;
        s += fk_map(truth.u, frame_same ? P : 100.0f - P, frame_same ? D : -D);
      }
      std::printf("truth_fk_%s=%.4f\n", dir == 0 ? "fwd" : "rev", s / m / gain_abs);
    }
    std::printf("truth_b=%.6e\n", truth.u.b / gain_abs);
  }
  std::printf("nvs_hash_start=%u\nnvs_hash_end=%u\nnvs_writes=%d\n", nvs_hash0,
              fnv(&nvs, sizeof(nvs)), nvs_writes);
  std::printf("runs=%d\n", static_cast<int>(runs.size()));
  std::printf("final_ident_state=%s\n", ident_state_name(servo.ident_state()));
  std::printf("final_outcome=%s\nfinal_reason=%s\n",
              OUTCOME_NAMES[static_cast<int>(servo.ident_outcome())],
              ident_reason_name(servo.ident_fail_reason()));
  std::printf("commit_seq=%u\n", servo.ident_commit_seq());
  // per-run results: the last run's working copy (== committed on OK)
  const IdentResult &w = servo.ident().work();
  const IdentDetail &d = w.detail;
  const UnitCalib &c = w.calib;
  std::printf("dir_sign=%d\nraw_at_0=%.2f\nraw_at_100=%.2f\n", c.dir_sign, c.raw_at_0,
              c.raw_at_100);
  std::printf("fs_p95=%.4f\nfs_min=%.4f\nfs_med=%.4f\n", d.fs_p95, d.fs_min, d.fs_med);
  for (int dir = 0; dir < 2; dir++)
    for (int k = 0; k < IDENT_POSITIONS; k++) {
      std::printf("fs_%s_%d=%.4f\n", dir == 0 ? "fwd" : "rev", k, d.fs[dir][k]);
      for (int r = 0; r < IDENT_REPS; r++)
        std::printf("fsrep_%s_%d_%d=%.4f\n", dir == 0 ? "fwd" : "rev", k, r,
                    d.fs_rep[dir][k][r]);
    }
  std::printf("impulse_duty=%.4f\nu_max=%.4f\nv_min_cmd=%.2f\n", c.impulse_duty, c.u_max,
              c.v_min_cmd);
  std::printf("fk_fwd=%.4f\nfk_rev=%.4f\nb_fwd=%.6e\nb_rev=%.6e\n", c.fk_mid_fwd,
              c.fk_mid_rev, c.b_fwd, c.b_rev);
  std::printf("flight_d1=%.4f\nflight_d2=%.4f\n", d.flight_duty[0], d.flight_duty[1]);
  std::printf("sigma0=%.3f\nsigma_pwm=%.3f\nspike_rate=%.5f\n", d.sigma0_counts,
              d.sigma_pwm_counts, d.spike_rate);
  std::printf("stall_duty=%.4f\nstall_motion=%.3f\nrail_duty=%.3f\nvm=%.3f\n", d.stall_duty,
              d.stall_motion_pct, d.rail_duty_used, d.vm);
  for (int dir = 0; dir < 2; dir++)
    for (int k = 0; k < IDENT_IMP_WIDTHS; k++) {
      const float wdt = icfg.imp_widths_ms[k];
      const float D = dir == 0 ? 1.0f : -1.0f;
      const float tt = truth_pulse(truth, gain_abs, frame_same ? 50.0f : 50.0f,
                                   (frame_same ? D : -D), c.impulse_duty,
                                   static_cast<int>(wdt + 0.5f));
      std::printf("imp_%s_%d=%.4f\ntruth_imp_%s_%d=%.4f\n", dir == 0 ? "fwd" : "rev", k,
                  d.imp_table[dir][k], dir == 0 ? "fwd" : "rev", k, tt);
    }
  for (int ph = 1; ph <= 9; ph++) std::printf("phase_ticks_%d=%u\n", ph, d.phase_ticks[ph]);
  std::printf("total_ticks=%u\n", d.total_ticks);
  for (size_t k = 0; k < runs.size(); k++) {
    const Run &r = runs[k];
    const size_t q = k + 1;
    std::printf("run%zu_start=%d\nrun%zu_end=%d\nrun%zu_post_u_max=%.4f\n", q, r.start, q,
                r.end, q, r.post_u_max);
    std::printf("run%zu_outcome=%s\nrun%zu_reason=%s\n", q, OUTCOME_NAMES[r.outcome], q,
                ident_reason_name(static_cast<IdentReason>(r.reason)));
    for (int ph = 1; ph <= 9; ph++) std::printf("run%zu_enter_%d=%d\n", q, ph, r.enter[ph]);
    const IdentDetail &rd = r.res.detail;
    const UnitCalib &rc = r.res.calib;
    std::printf("run%zu_fs_p95=%.4f\nrun%zu_b_fwd=%.6e\nrun%zu_b_rev=%.6e\n", q, rd.fs_p95,
                q, rc.b_fwd, q, rc.b_rev);
    std::printf("run%zu_impulse_duty=%.4f\nrun%zu_total_ticks=%u\n", q, rc.impulse_duty, q,
                rd.total_ticks);
    for (int dir = 0; dir < 2; dir++) {
      for (int p = 0; p < IDENT_POSITIONS; p++)
        std::printf("run%zu_fs_%s_%d=%.4f\n", q, dir == 0 ? "fwd" : "rev", p, rd.fs[dir][p]);
      for (int w = 0; w < IDENT_IMP_WIDTHS; w++)
        std::printf("run%zu_imp_%s_%d=%.4f\n", q, dir == 0 ? "fwd" : "rev", w,
                    rd.imp_table[dir][w]);
    }
  }
  std::printf("moves=%d\n", static_cast<int>(moves.size()));
  for (size_t k = 0; k < moves.size(); k++) {
    Move &m = moves[k];
    if (m.settled < 0) m.final_frame = frame(plant.p());
    std::printf("move%zu_target=%.3f\nmove%zu_final=%.4f\nmove%zu_settled=%d\n"
                "move%zu_issued=%d\nmove%zu_flag=%s\nmove%zu_pulse_u_max=%.4f\n"
                "move%zu_u_max=%.4f\nmove%zu_move_ticks=%d\nmove%zu_status=%d\n",
                k + 1, m.target, k + 1, m.final_frame, k + 1, m.settled, k + 1, m.issued,
                k + 1, m.flag.c_str(), k + 1, m.pulse_u_max, k + 1, m.u_max, k + 1,
                m.move_ticks, k + 1, m.status);
  }
  std::printf("ident_transitions=");
  for (int a = 0; a < 12; a++)
    for (int b = 0; b < 12; b++)
      if (trans[a][b]) std::printf("%d>%d:%d,", a, b, trans[a][b]);
  std::printf("\nctrl_violation=%d\nrejected_during=%d\naccepted_during=%d\n"
              "starts_during=%d\n", ctrl_violation, rej_during, acc_during, starts_during);
  std::printf("grab_detect_post=%d\nident_done_tick=%d\n", grab_detect_post, ident_done_tick);
  std::printf("final_identified=%d\nfinal_derived=%d\n", servo.calib().identified ? 1 : 0,
              servo.calib().derived ? 1 : 0);
  std::printf("final_pos_frame=%.4f\n", frame(plant.p()));
  return 0;
}
