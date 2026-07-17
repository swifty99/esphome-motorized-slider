// sim_runner — SIL CLI (plan.md Phase 5): links the servo core against the
// C++ port of the validated plant sim, plays a scenario file, emits a CSV
// trace + a key=value summary on stdout. pytest asserts the tripwires on the
// output; --bug N arms one seeded bug for the negative controls.
//
// Scenario format (one directive per line, '#' comments):
//   p0 30.0
//   perturb <d_fwd> <d_rev> <d_m> <d_b>
//   seed 42
//   duration 8.0            # seconds hard cap
//   rest 2.5                # stop this long after the first settle (0 = full)
//   at <t> target <pct> [speed <pct_per_s>]
//   at <t> grab <dur_s>     # rigid hand holds the slider still
//   at <t> drag <to_pct> <dur_s>   # hand drags the slider along a ramp
//   at <t> step <delta_pct> # instant true-position offset (T5)
//   at <t> release | abort on|off | freeze on|off

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

struct Event {
  int tick = 0;
  std::string kind;
  float a = 0.0f, b = 0.0f;
  bool flag = false;
};

struct Scenario {
  float p0 = 50.0f;
  PlantPerturb perturb;
  uint32_t seed = 1;
  float duration_s = 8.0f;
  float rest_s = 2.5f;
  int recorder = 0;
  int impulse_max = -1;  // config overrides (-1 = keep §5 default)
  float tol = -1.0f;
  float crawl_step = -1.0f;   // self-adjusting crawl step (<0 = keep default 0)
  int crawl_settle_ms = -1;   // crawl-exclusive settle (<0 = keep default)
  float crawl_land_zone = -1.0f;  // crawl landing zone (<0 = keep default 0)
  std::vector<Event> events;
};

bool parse_scenario(const char *path, Scenario &sc) {
  std::ifstream f(path);
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    const auto h = line.find('#');
    if (h != std::string::npos) line = line.substr(0, h);
    std::istringstream ss(line);
    std::string tok;
    if (!(ss >> tok)) continue;
    if (tok == "p0") ss >> sc.p0;
    else if (tok == "seed") ss >> sc.seed;
    else if (tok == "duration") ss >> sc.duration_s;
    else if (tok == "rest") ss >> sc.rest_s;
    else if (tok == "recorder") ss >> sc.recorder;
    else if (tok == "perturb")
      ss >> sc.perturb.d_fwd >> sc.perturb.d_rev >> sc.perturb.d_m >> sc.perturb.d_b;
    else if (tok == "config") {
      std::string name;
      ss >> name;
      if (name == "impulse_max") ss >> sc.impulse_max;
      else if (name == "tol") ss >> sc.tol;
      else if (name == "crawl_step") ss >> sc.crawl_step;
      else if (name == "crawl_settle_ms") ss >> sc.crawl_settle_ms;
      else if (name == "crawl_land_zone") ss >> sc.crawl_land_zone;
      else {
        std::fprintf(stderr, "scenario: unknown config '%s'\n", name.c_str());
        return false;
      }
    }
    else if (tok == "at") {
      float t;
      Event e;
      ss >> t >> e.kind;
      e.tick = static_cast<int>(t * 1000.0f + 0.5f);
      if (e.kind == "target") {
        ss >> e.a;
        std::string k2;
        if (ss >> k2 && k2 == "speed") ss >> e.b;
      } else if (e.kind == "grab") ss >> e.a;
      else if (e.kind == "drag") ss >> e.a >> e.b;
      else if (e.kind == "step") ss >> e.a;
      else if (e.kind == "abort" || e.kind == "freeze") {
        std::string k2;
        ss >> k2;
        e.flag = (k2 == "on");
      }
      sc.events.push_back(e);
    } else {
      std::fprintf(stderr, "scenario: unknown directive '%s'\n", tok.c_str());
      return false;
    }
  }
  return true;
}

const char *STATE_NAMES[] = {"IDLE", "MOVE", "BRAKE", "PULSE",
                             "WAIT", "CRAWL", "RELEASED"};
const char *FLAG_NAMES[] = {"none", "reached", "gave_up", "adopted"};
const char *REASON_NAMES[] = {"none", "grab", "interlock", "watchdog",
                              "fault", "command"};

}  // namespace

int main(int argc, char **argv) {
  const char *scenario_path = nullptr;
  const char *out_path = nullptr;
  int bug = 0;
  int recorder_override = -1;
  long seed_override = -1;
  for (int i = 1; i < argc; i++) {
    if (!std::strcmp(argv[i], "--out") && i + 1 < argc) out_path = argv[++i];
    else if (!std::strcmp(argv[i], "--bug") && i + 1 < argc) bug = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--recorder") && i + 1 < argc)
      recorder_override = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--seed") && i + 1 < argc)
      seed_override = std::atol(argv[++i]);
    else scenario_path = argv[i];
  }
  if (!scenario_path) {
    std::fprintf(stderr,
                 "usage: sim_runner <scenario> [--out csv] [--bug N] "
                 "[--recorder N] [--seed N]\n");
    return 2;
  }
  Scenario sc;
  if (!parse_scenario(scenario_path, sc)) {
    std::fprintf(stderr, "cannot parse scenario %s\n", scenario_path);
    return 2;
  }
  if (recorder_override >= 0) sc.recorder = recorder_override;
  if (seed_override >= 0) sc.seed = static_cast<uint32_t>(seed_override);

  UnitCalib nominal;  // per-unit defaults = identified snapshot
  ServoConfig cfg;
  if (sc.impulse_max > 0) cfg.impulse_max = sc.impulse_max;
  if (sc.tol > 0.0f) cfg.tol = sc.tol;
  if (sc.crawl_step >= 0.0f) cfg.crawl_step = sc.crawl_step;
  if (sc.crawl_settle_ms >= 0) cfg.crawl_settle_ms = sc.crawl_settle_ms;
  if (sc.crawl_land_zone >= 0.0f) cfg.crawl_land_zone = sc.crawl_land_zone;
  TruthPlant truth = TruthPlant::from(nominal, sc.perturb);

  PlantStepper plant;
  plant.init(&truth, sc.p0);
  SensorSim sensor;
  sensor.init(&nominal, sc.seed);
  Servo servo;
  servo.init(&nominal, &cfg, sc.p0, sc.recorder,
             static_cast<SeededBug>(bug));

  std::FILE *csv = nullptr;
  if (out_path) {
    csv = std::fopen(out_path, "w");
    if (!csv) {
      std::fprintf(stderr, "cannot open %s\n", out_path);
      return 2;
    }
    std::fprintf(csv,
                 "tick,pos,vel,stuck,z,p_hat,v_hat,u_cmd,u,state,flag,reason,"
                 "clamped,v_des,ff,cmd,obs_stuck,rejects,p00\n");
  }

  const int n_max = static_cast<int>(sc.duration_s * 1000.0f);
  const int rest_ticks = static_cast<int>(sc.rest_s * 1000.0f);

  // hand state
  int grab_until = -1;
  float grab_pos = 0.0f;
  int drag_until = -1, drag_start = 0;
  float drag_from = 0.0f, drag_to = 0.0f;

  size_t next_event = 0;
  int first_cmd_tick = -1;
  int settled_at = -1;

  for (int i = 0; i < n_max; i++) {
    // commands fire before the tick (estimate = last tick's posterior)
    int cmd_marker = 0;
    while (next_event < sc.events.size() && sc.events[next_event].tick <= i) {
      const Event &e = sc.events[next_event++];
      if (e.kind == "target") {
        servo.set_target(e.a, e.b);
        if (first_cmd_tick < 0) first_cmd_tick = i;
        cmd_marker = 1;
      } else if (e.kind == "release") {
        servo.release();
        cmd_marker = 2;
      } else if (e.kind == "abort") {
        servo.set_abort(e.flag);
        cmd_marker = 3;
      } else if (e.kind == "freeze") {
        sensor.set_frozen(e.flag, plant.p());
      } else if (e.kind == "grab") {
        grab_until = i + static_cast<int>(e.a * 1000.0f);
        grab_pos = plant.p();
      } else if (e.kind == "drag") {
        drag_start = i;
        drag_until = i + static_cast<int>(e.b * 1000.0f);
        drag_from = plant.p();
        drag_to = e.a;
      } else if (e.kind == "step") {
        plant.force(plant.p() + e.a, 0.0f, true);
      }
    }

    const float z = sensor.sample(plant.p(), servo.u_prev());
    const float u = servo.tick(z);
    plant.step(u);

    // the hand is infinitely stiff (stated limitation, silharness)
    if (grab_until >= 0 && i < grab_until) {
      plant.force(grab_pos, 0.0f, true);
    } else if (drag_until >= 0 && i < drag_until) {
      const float frac = static_cast<float>(i - drag_start) /
                         static_cast<float>(drag_until - drag_start);
      const float pos = drag_from + (drag_to - drag_from) * frac;
      const float vel = (drag_to - drag_from) /
                        (static_cast<float>(drag_until - drag_start) * 0.001f);
      plant.force(pos, vel, false);
    } else if (drag_until >= 0 && i == drag_until) {
      plant.force(plant.p(), 0.0f, true);
      drag_until = -1;
    }

    const Controller &c = servo.controller();
    if (csv) {
      std::fprintf(
          csv,
          "%d,%.4f,%.3f,%d,%.4f,%.4f,%.3f,%.4f,%.4f,%s,%s,%s,%d,%.2f,%.4f,%d,"
          "%d,%d,%.3e\n",
          i, plant.p(), plant.v(), plant.stuck() ? 1 : 0, z,
          servo.observer().p_hat(), servo.observer().v_hat(),
          servo.u_cmd_last(), u, STATE_NAMES[static_cast<int>(c.state())],
          FLAG_NAMES[static_cast<int>(c.settle_flag())],
          REASON_NAMES[static_cast<int>(c.release_reason())],
          servo.safety().clamped() ? 1 : 0, c.last_v_des(), c.last_ff(),
          cmd_marker, servo.observer().model_stuck() ? 1 : 0,
          servo.observer().gate_rejects(), servo.observer().p00());
    }

    if (first_cmd_tick >= 0 && settled_at < 0 && c.settled() &&
        next_event >= sc.events.size())
      settled_at = i;
    if (settled_at >= 0 && rest_ticks > 0 && i - settled_at >= rest_ticks)
      break;
  }

  if (csv) std::fclose(csv);

  const Controller &c = servo.controller();
  const ControllerLog &log = c.log();
  std::printf("settled_tick=%d\n", settled_at);
  std::printf("final_state=%s\n", STATE_NAMES[static_cast<int>(c.state())]);
  std::printf("settle_flag=%s\n", FLAG_NAMES[static_cast<int>(c.settle_flag())]);
  std::printf("release_reason=%s\n",
              REASON_NAMES[static_cast<int>(c.release_reason())]);
  std::printf("final_pos=%.4f\n", plant.p());
  std::printf("final_p_hat=%.4f\n", servo.observer().p_hat());
  std::printf("target=%.4f\n", c.target());
  std::printf("hold=%.4f\n", c.hold());
  std::printf("pulses=%d\nkicks=%d\novershoots=%d\nremoves=%d\n", log.pulses,
              log.kicks, log.overshoots, log.removes);
  std::printf("gave_up=%d\ncrawl_degraded=%d\ntarget_clamped=%d\n",
              log.gave_up ? 1 : 0, log.crawl_degraded ? 1 : 0,
              log.target_clamped ? 1 : 0);
  std::printf("watchdog_tripped=%d\ngrab_detect_tick=%d\nsettle_latch_tick=%d\n",
              log.watchdog_tripped ? 1 : 0, log.grab_detect_tick,
              log.settled_tick);
  std::printf("observer_fault=%d\n", servo.observer().fault() ? 1 : 0);
  return 0;
}
