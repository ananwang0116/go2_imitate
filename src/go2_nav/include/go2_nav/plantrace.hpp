// Plan trace: numbers that say WHY a replan produced a different path (no ROS here, tested offline with gtest).
// mot_nav replans about once a second. The path may change because (a) the goal / a hit ahead / a blocked path forced it,
// (b) the fresh scan points changed the costmap near the path, (c) /per_nav_map changed (evidence cells flipped) near the path,
// (d) the dog's pose jumped (map->odom or SLAM), (e) the relay was dropped, (f) nothing changed and A* / the smoother just
// picked another equal path. `explain_change` picks the first cause that the numbers support, in that order of strength.
#pragma once
#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "go2_nav/types.hpp"

namespace go2_nav {

// Distance from p to the polyline `path` (to the nearest segment); +inf for an empty path.
inline double dist_to_path(Vec2 p, const std::vector<Vec2>& path) {
  double best = std::numeric_limits<double>::infinity();
  if (path.size() == 1) return hyp(p.x - path[0].x, p.y - path[0].y);
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const double ax = path[i].x, ay = path[i].y, dx = path[i + 1].x - ax, dy = path[i + 1].y - ay;
    const double l2 = dx * dx + dy * dy;
    const double t = l2 < 1e-12 ? 0.0 : clampd(((p.x - ax) * dx + (p.y - ay) * dy) / l2, 0.0, 1.0);
    best = std::min(best, hyp(p.x - (ax + t * dx), p.y - (ay + t * dy)));
  }
  return best;
}

// The first `len_m` metres of a path (measured along it), the last point cut exactly.
inline std::vector<Vec2> head_of(const std::vector<Vec2>& path, double len_m) {
  std::vector<Vec2> out;
  double s = 0;
  for (size_t i = 0; i < path.size(); ++i) {
    if (i == 0) {
      out.push_back(path[0]);
      continue;
    }
    const double seg = hyp(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    if (s + seg >= len_m) {
      const double f = seg < 1e-9 ? 0.0 : (len_m - s) / seg;
      out.push_back({path[i - 1].x + f * (path[i].x - path[i - 1].x), path[i - 1].y + f * (path[i].y - path[i - 1].y)});
      return out;
    }
    s += seg;
    out.push_back(path[i]);
  }
  return out;
}

// Largest distance of any point of `a` (the new path, resampled every 0.1 m) from the polyline `b` (the old path), over the first
// `len_m` metres of `a`. 0 when b is empty (nothing to compare with).
inline double path_deviation(const std::vector<Vec2>& a, const std::vector<Vec2>& b, double len_m) {
  if (a.size() < 2 || b.size() < 2) return 0.0;
  const std::vector<Vec2> h = head_of(a, len_m);
  double worst = 0;
  for (size_t i = 0; i + 1 < h.size(); ++i) {
    const double seg = hyp(h[i + 1].x - h[i].x, h[i + 1].y - h[i].y);
    const int n = std::max(1, static_cast<int>(seg / 0.1));
    for (int k = 0; k <= n; ++k) {
      const double f = static_cast<double>(k) / n;
      worst = std::max(worst, dist_to_path({h[i].x + f * (h[i + 1].x - h[i].x), h[i].y + f * (h[i + 1].y - h[i].y)}, b));
    }
  }
  return worst;
}

// Heading (rad) of the path over its first `len_m` metres (chord from the start to the cut point); 0 for a path under 2 points.
inline double head_heading(const std::vector<Vec2>& path, double len_m) {
  const std::vector<Vec2> h = head_of(path, len_m);
  if (h.size() < 2) return 0.0;
  return std::atan2(h.back().y - h.front().y, h.back().x - h.front().x);
}

// Nearest of `pts` to the polyline `path` within its first `len_m` metres; inf when none.
inline double nearest_point_to_path(const std::vector<Vec2>& pts, const std::vector<Vec2>& path, double len_m) {
  double best = std::numeric_limits<double>::infinity();
  const std::vector<Vec2> h = head_of(path, len_m);
  if (h.size() < 2) return best;
  for (const Vec2& p : pts) best = std::min(best, dist_to_path(p, h));
  return best;
}

struct PlanDiff {
  double deviation_m = 0;          // new path vs old path over the first `horizon_m`
  double head_turn_deg = 0;        // change of the first metre's heading
  double length_old_m = 0, length_new_m = 0;
};

inline PlanDiff diff_paths(const std::vector<Vec2>& old_path, const std::vector<Vec2>& new_path, double horizon_m) {
  PlanDiff d;
  d.deviation_m = path_deviation(new_path, old_path, horizon_m);
  if (old_path.size() >= 2 && new_path.size() >= 2) d.head_turn_deg = wrap(head_heading(new_path, 1.0) - head_heading(old_path, 1.0)) * 180.0 / M_PI;
  auto len = [](const std::vector<Vec2>& p) {
    double s = 0;
    for (size_t i = 1; i < p.size(); ++i) s += hyp(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
    return s;
  };
  d.length_old_m = len(old_path);
  d.length_new_m = len(new_path);
  return d;
}

// What was different between two consecutive plans, as numbers measured at the OLD path's first `near_m` metres.
struct PlanCause {
  std::string forced;              // why this plan was not periodic: goal / blocked / hit / plan_fail / no_pose ("" = periodic)
  double pose_jump_m = 0;          // how far the dog's map pose moved since the last plan beyond what odometry moved (0 if unknown)
  int overlay_new_cells = 0;       // cells the fresh scan points blocked that the map did not
  double overlay_near_old_m = std::numeric_limits<double>::infinity();   // nearest such cell to the old path's head
  int map_flips = 0;               // blocked/free flips of /per_nav_map since the last plan
  double map_flip_near_old_m = std::numeric_limits<double>::infinity();  // nearest flipped cell to the old path's head
  bool relay_dropped = false;      // the old path existed, the relay was not used
  std::string relay_why;           // why (off_path / almost_done / kept_blocked / astar_from_relay_failed / ...)
  bool smooth_skipped = false;
};

// One word for the likeliest reason a path changed, from the numbers; thresholds are the same ones mot_nav uses to report.
inline std::string explain_change(const PlanCause& c, double near_m = 1.5) {
  if (!c.forced.empty() && c.forced != "hit") return "forced:" + c.forced;
  if (c.pose_jump_m > 0.3) return "pose_jump";
  if (c.overlay_new_cells > 0 && c.overlay_near_old_m < near_m) return "scan_overlay";
  if (c.map_flips > 0 && c.map_flip_near_old_m < near_m) return "map_change";
  if (c.forced == "hit") return "forced:hit";
  if (c.relay_dropped) return "relay_dropped:" + (c.relay_why.empty() ? std::string("unknown") : c.relay_why);
  if (c.smooth_skipped) return "smooth_skipped";
  return "unexplained";
}

}  // namespace go2_nav
