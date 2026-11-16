// Plan relay (SCAN-Planner's "replan from the current trajectory", docs/05 section 9, item 5).
// A replan used to start at the dog and rebuild the whole path, so the first metres could change shape every second and the
// follower swerved. With a relay the near part of the OLD path is kept as it is (as long as it is still free and the dog is on it)
// and only the part beyond the relay point is planned again; the new path starts where the old one says the dog is going to be.
//   new path = [dog] + old path from the dog's nearest point up to `relay_m` ahead + A* from there to the goal.
// Falls back to a plain plan from the dog (relayed = false) when: no old path, the dog is more than `max_off_m` off it, the old
// path is nearly finished, the kept stretch is blocked now, or A* from the relay point finds nothing.
#pragma once
#include "go2_nav/pathcheck.hpp"

namespace go2_nav {

struct RelayParams {
  double relay_m = 1.0;      // length of old path that is kept (about the follower's look-ahead)
  double max_off_m = 0.4;    // farther than this from the old path: plan from the dog
};

struct RelayPlan {
  bool ok = false, relayed = false;
  std::string reason;
  std::string why;                       // when relayed is false although an old path was given: no_old_path / off_path / almost_done / kept_blocked / relay_astar_failed
  std::vector<Vec2> path;
  double length_m = 0.0, kept_m = 0.0;   // kept_m = length of old path that was kept
  int expanded = 0;
};

inline RelayPlan plan_relay(const CostMap& cm, const PlanParams& pp, const RelayParams& rp, const std::vector<Vec2>& old, Vec2 robot, Vec2 goal,
                            double radius) {
  RelayPlan out;
  auto plain = [&](const char* why) {
    out.why = why;
    Planner pl(cm, pp);
    const PlanResult r = pl.plan(robot, goal, radius);
    out.ok = r.ok;
    out.reason = r.reason;
    out.path = r.path;
    out.length_m = r.length_m;
    out.expanded = r.expanded;
    out.relayed = false;
    out.kept_m = 0;
    return out;
  };
  if (old.size() < 3) return plain("no_old_path");
  size_t i0 = 0;
  double bd = 1e18;
  for (size_t i = 0; i < old.size(); ++i) {
    const double d = hyp(old[i].x - robot.x, old[i].y - robot.y);
    if (d < bd) {
      bd = d;
      i0 = i;
    }
  }
  if (bd > rp.max_off_m) return plain("off_path");
  size_t ir = i0;
  double kept = 0;
  while (ir + 1 < old.size() && kept < rp.relay_m) {
    kept += hyp(old[ir + 1].x - old[ir].x, old[ir + 1].y - old[ir].y);
    ++ir;
  }
  if (kept < 0.5 * rp.relay_m || ir + 1 >= old.size()) return plain("almost_done");           // the old path is almost done: nothing to keep
  // the kept stretch must still be free (the map here already contains the fresh scan points)
  for (size_t i = i0; i < ir; ++i) {
    const Vec2 a = old[i], b = old[i + 1];
    const double yaw = std::atan2(b.y - a.y, b.x - a.x);
    if (hyp(a.x - robot.x, a.y - robot.y) <= pp.start_relax_m) continue;          // close to the dog its own clearance rules
    const double d = cm.footprint_clearance(a.x, a.y, yaw, pp.fp_offset_m);
    if (std::isnan(d) || d < pp.hard_m * 0.9) return plain("kept_blocked");
  }
  Planner pl(cm, pp);
  const PlanResult r = pl.plan(old[ir], goal, radius);
  if (!r.ok || r.path.size() < 2) return plain("relay_astar_failed");
  out.path.push_back(robot);
  for (size_t i = i0; i <= ir; ++i)
    if (hyp(old[i].x - out.path.back().x, old[i].y - out.path.back().y) > 1e-6) out.path.push_back(old[i]);
  for (size_t i = 1; i < r.path.size(); ++i) out.path.push_back(r.path[i]);
  out.ok = out.relayed = true;
  out.length_m = path_length(out.path);
  out.kept_m = kept;
  out.expanded = r.expanded;
  return out;
}

}  // namespace go2_nav
