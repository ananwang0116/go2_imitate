// Path goals: follow a taught polyline as it is (no A* on it). Join it when the dog is off it, detour when a stretch is blocked.
#pragma once
#include "go2_nav/planner.hpp"

namespace go2_nav {

struct TaughtPlan {
  bool ok = false;
  std::string reason;                  // no_path | ...
  std::vector<Vec2> path;              // what the follower follows: [join / detour leg] + the rest of the taught path
  bool joined = false, detoured = false;
  size_t k0 = 0;                       // nearest taught index (monotone: a loop that passes the start again never jumps)
};

struct TaughtParams {
  double join_m = 0.6;                 // farther than this from the path: A* to a point just ahead on it
  double join_ahead_m = 0.5;
  double check_ahead_m = 3.0;          // a fresh point on this much of the path ahead = blocked
  double detour_after_m = 0.8;         // rejoin the path this far beyond the last blocked point
  size_t window = 80;                  // search window for the nearest index [pts]
};

inline size_t nearest_index(const std::vector<Vec2>& p, Vec2 q, size_t from, size_t window) {
  size_t best = std::min(from, p.size() - 1);
  double bd = 1e18;
  for (size_t i = from; i < p.size() && i < from + window; ++i) {
    const double d = hyp(p[i].x - q.x, p[i].y - q.y);
    if (d < bd) {
      bd = d;
      best = i;
    }
  }
  return best;
}

// cm: the costmap WITH the fresh scan points blocked (the planner sees them); fresh: the same points (map frame)
inline TaughtPlan plan_on_taught(const CostMap& cm, const PlanParams& pp, const TaughtParams& tp, const std::vector<Vec2>& taught,
                                 Vec2 robot, size_t& hint, const std::vector<Vec2>& fresh, double hw, double snap_m) {
  TaughtPlan r;
  if (taught.size() < 2) {
    r.reason = "no_path";
    return r;
  }
  const size_t n = taught.size();
  size_t k0 = nearest_index(taught, robot, hint, tp.window);
  hint = k0;
  r.k0 = k0;
  // blocked stretch ahead? (a fresh point on the path, or the path leaves the passable cells)
  size_t last_bad = n;                 // n = none
  double acc = 0;
  for (size_t i = k0 + 1; i < n && acc < tp.check_ahead_m; ++i) {
    acc += hyp(taught[i].x - taught[i - 1].x, taught[i].y - taught[i - 1].y);
    bool bad = cm.clearance(taught[i].x, taught[i].y) < pp.hard_m * 0.8;
    for (size_t q = 0; q < fresh.size() && !bad; ++q) bad = hyp(fresh[q].x - taught[i].x, fresh[q].y - taught[i].y) < hw + 0.05;
    if (bad) last_bad = i;
  }
  const double d0 = hyp(robot.x - taught[k0].x, robot.y - taught[k0].y);
  if (d0 <= tp.join_m && last_bad == n) {
    r.path.push_back(robot);
    r.path.insert(r.path.end(), taught.begin() + static_cast<long>(k0), taught.end());
    r.ok = true;
    return r;
  }
  size_t kt;                           // where the A* leg ends and the taught path continues
  if (last_bad != n) {
    kt = last_bad;
    double beyond = 0;
    while (kt + 1 < n && beyond < tp.detour_after_m) {
      beyond += hyp(taught[kt + 1].x - taught[kt].x, taught[kt + 1].y - taught[kt].y);
      ++kt;
    }
    while (kt + 1 < n && cm.clearance(taught[kt].x, taught[kt].y) < pp.hard_m) ++kt;       // rejoin where it is free again
    r.detoured = true;
  } else {
    kt = k0;
    double ahead = 0;
    while (kt + 1 < n && ahead < tp.join_ahead_m) {
      ahead += hyp(taught[kt + 1].x - taught[kt].x, taught[kt + 1].y - taught[kt].y);
      ++kt;
    }
    r.joined = true;
  }
  Planner planner(cm, pp);
  const PlanResult leg = planner.plan(robot, taught[kt], snap_m);
  if (!leg.ok) {
    r.reason = leg.reason;
    return r;
  }
  r.path = leg.path;
  if (kt + 1 < n) r.path.insert(r.path.end(), taught.begin() + static_cast<long>(kt + 1), taught.end());
  r.ok = true;
  return r;
}

}  // namespace go2_nav
