// Look ahead along the path that is being followed: where, if at all, would the body hit something (SCAN-Planner's safety timer,
// docs/05 section 9, item 2). The answer is a DISTANCE along the path; mot_nav turns it into a time (distance / speed) to decide
// "replan now" and "slow down", instead of the old "any scan point within 3 m of the path" rule that knew nothing about how soon.
// Two things are checked at every sample (heading = the path direction there):
//   - the map's blocked cells with the two-point footprint (not near the robot: there the dog's own clearance rules; not in the
//     last `tail_free_m`: the goal area, where the dog may stand close to a wall);
//   - the fresh scan points against the same two footprint centres (always).
#pragma once
#include "go2_nav/planner.hpp"

namespace go2_nav {

struct PathHit {
  bool hit = false;
  double dist_m = 1e9;       // along the path from the robot to the first sample that hits
};

struct PathCheckParams {
  double horizon_m = 3.0;    // how far ahead to look
  double step_m = 0.1;
  double near_m = 0.6;       // no map check this close to the robot (start_relax_m)
  double map_factor = 0.9;   // a sample hits the map when its footprint clearance < hard_m * map_factor
  double scan_r_m = 0.25;    // a fresh point within this of a footprint centre hits (half width + 0.05)
  double tail_free_m = 0.0;  // no map check within this much path length of the end
};

inline PathHit first_hit(const CostMap* cm, const PlanParams& pp, const PathCheckParams& pc, const std::vector<Vec2>& path, Vec2 robot,
                         const std::vector<Vec2>& fresh) {
  PathHit out;
  if (path.size() < 2) return out;
  size_t k0 = 0;
  double bd = 1e18;
  for (size_t i = 0; i < path.size(); ++i) {
    const double d = hyp(path[i].x - robot.x, path[i].y - robot.y);
    if (d < bd) {
      bd = d;
      k0 = i;
    }
  }
  double total_rest = 0;
  for (size_t i = k0 + 1; i < path.size(); ++i) total_rest += hyp(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  double s = bd;                         // arc length from the robot (first the way to the nearest path point)
  double to_go = total_rest;
  const double scan_r2 = pc.scan_r_m * pc.scan_r_m;
  for (size_t i = k0; i + 1 < path.size() && s < pc.horizon_m; ++i) {
    const Vec2 a = path[i], b = path[i + 1];
    const double len = hyp(b.x - a.x, b.y - a.y);
    if (len < 1e-9) continue;
    const double yaw = std::atan2(b.y - a.y, b.x - a.x), cx = std::cos(yaw) * pp.fp_offset_m, cy = std::sin(yaw) * pp.fp_offset_m;
    const int n = std::max(1, static_cast<int>(std::ceil(len / pc.step_m)));
    for (int k = 0; k < n; ++k) {
      const double t = static_cast<double>(k) / n;
      const Vec2 p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
      const double here = s + len * t;
      if (here >= pc.horizon_m) return out;
      bool bad = false;
      if (cm && cm->valid() && here > pc.near_m && to_go - len * t > pc.tail_free_m) {
        const double d = cm->footprint_clearance(p.x, p.y, yaw, pp.fp_offset_m);
        bad = std::isnan(d) || d < pp.hard_m * pc.map_factor;
      }
      for (size_t q = 0; q < fresh.size() && !bad; ++q) {
        const double fx = fresh[q].x, fy = fresh[q].y;
        bad = (fx - p.x - cx) * (fx - p.x - cx) + (fy - p.y - cy) * (fy - p.y - cy) < scan_r2 ||
              (fx - p.x + cx) * (fx - p.x + cx) + (fy - p.y + cy) * (fy - p.y + cy) < scan_r2;
      }
      if (bad) {
        out.hit = true;
        out.dist_m = here;
        return out;
      }
    }
    s += len;
    to_go -= len;
  }
  return out;
}

}  // namespace go2_nav
