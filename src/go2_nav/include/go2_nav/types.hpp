// Basic types of mot_nav (no ROS here: everything in this directory is tested offline with gtest).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace go2_nav {

struct Vec2 {
  double x = 0, y = 0;
};
struct Pose2 {
  double x = 0, y = 0, yaw = 0;
};
struct Cmd {
  double vx = 0, vy = 0, wz = 0;   // base_footprint frame
};

inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
inline double hyp(double a, double b) { return std::sqrt(a * a + b * b); }
inline std::string fmt_num(double v) {
  char b[32];
  std::snprintf(b, sizeof b, "%.2f", v);
  return b;
}

// Occupancy grid in the same convention as /per_nav_map: row j = y index (bottom-up), -1 unknown, 0..100 cost.
// BLOCKED (>= 65): an obstacle the laser has confirmed, or a wall drawn by hand in the map editor (glass) - never entered.
// PRIOR (50..64): a wall of the saved map that nobody has looked at in this run (per_evidence writes 60). It may have been a
// thing that stood there while mapping and has gone since, so it is NOT blocked: the planner keeps away from it and only
// goes through it when there is no reasonable way round; the laser then decides (user decision 2026-10-26, P4).
struct Grid {
  int w = 0, h = 0;
  double res = 0.05, ox = 0.0, oy = 0.0;
  std::vector<int8_t> data;
  static constexpr int BLOCKED = 65;
  static constexpr int PRIOR = 50;

  Grid() = default;
  Grid(int w_, int h_, double res_, double ox_, double oy_, int8_t fill = 0)
      : w(w_), h(h_), res(res_), ox(ox_), oy(oy_), data(static_cast<size_t>(w_) * h_, fill) {}
  bool inside(int i, int j) const { return i >= 0 && j >= 0 && i < w && j < h; }
  size_t idx(int i, int j) const { return static_cast<size_t>(j) * w + i; }
  void cell(double x, double y, int& i, int& j) const {
    i = static_cast<int>(std::floor((x - ox) / res));
    j = static_cast<int>(std::floor((y - oy) / res));
  }
  Vec2 center(int i, int j) const { return {ox + (i + 0.5) * res, oy + (j + 0.5) * res}; }
  bool blocked(int i, int j) const { return data[idx(i, j)] >= BLOCKED; }
  static bool is_prior(int8_t v) { return v >= PRIOR && v < BLOCKED; }
};

}  // namespace go2_nav
