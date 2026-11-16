// Costmap: /per_nav_map (prior fused with evidence) + fresh scan points -> exact distance to the nearest blocked cell, and a
// second distance to the nearest unconfirmed prior wall (types.hpp PRIOR).
// The "inflation" of Nav2 is not a layer here: the planner and the follower ask `clearance(x, y)` and turn it into a cost
// with the geometry distances (hard = body_half_width + safe_stop_gap, soft zone up to body_half_width + waypoint_clearance).
// Prior walls only ever cost (planner.hpp): `prior_clearance(x, y)`.
#pragma once
#include <limits>

#include "go2_nav/types.hpp"

namespace go2_nav {

// Exact squared Euclidean distance transform (Felzenszwalb & Huttenlocher, 2012), in cells^2. f[q] = 0 at sources, huge elsewhere.
inline void edt_1d(const double* f, int n, double* d, int* v, double* z) {
  const double INF = 1e30;
  int k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = INF;
  for (int q = 1; q < n; ++q) {
    auto cross = [&](int p) {
      return ((f[q] + static_cast<double>(q) * q) - (f[p] + static_cast<double>(p) * p)) / (2.0 * q - 2.0 * p);
    };
    double s = cross(v[k]);
    while (s <= z[k]) {          // z[0] = -INF: k never goes below 0
      --k;
      s = cross(v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = INF;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    const double dq = static_cast<double>(q - v[k]);
    d[q] = dq * dq + f[v[k]];
  }
}

inline std::vector<float> distance_m(const std::vector<uint8_t>& blocked, int w, int h, double res) {
  const double BIG = 1e15;
  std::vector<double> g(static_cast<size_t>(w) * h);
  for (size_t i = 0; i < g.size(); ++i) g[i] = blocked[i] ? 0.0 : BIG;
  const int n = std::max(w, h);
  std::vector<double> f(n), d(n), z(n + 1);
  std::vector<int> v(n);
  for (int i = 0; i < w; ++i) {                      // columns
    for (int j = 0; j < h; ++j) f[j] = g[static_cast<size_t>(j) * w + i];
    edt_1d(f.data(), h, d.data(), v.data(), z.data());
    for (int j = 0; j < h; ++j) g[static_cast<size_t>(j) * w + i] = d[j];
  }
  for (int j = 0; j < h; ++j) {                      // rows
    for (int i = 0; i < w; ++i) f[i] = g[static_cast<size_t>(j) * w + i];
    edt_1d(f.data(), w, d.data(), v.data(), z.data());
    for (int i = 0; i < w; ++i) g[static_cast<size_t>(j) * w + i] = d[i];
  }
  std::vector<float> out(g.size());
  for (size_t i = 0; i < g.size(); ++i) out[i] = g[i] >= BIG * 0.5 ? 1e6f : static_cast<float>(std::sqrt(g[i]) * res);
  return out;
}

struct CostMap {
  Grid grid;                       // the map as received (prior + evidence)
  std::vector<float> dist;         // [m] centre of a cell to the nearest blocked cell (1e6 = none)
  std::vector<float> prior_dist;   // [m] centre of a cell to the nearest unconfirmed prior wall (1e6 = none)
  bool valid() const { return grid.w > 0 && dist.size() == grid.data.size(); }

  // Distance from (x, y) to the nearest blocked cell; NaN outside the map.
  double clearance(double x, double y) const {
    int i, j;
    grid.cell(x, y, i, j);
    if (!grid.inside(i, j)) return std::numeric_limits<double>::quiet_NaN();
    return dist[grid.idx(i, j)];
  }
  // Two-point footprint (SCAN-Planner's "double cylinder", docs/05): the body is two discs of the hard radius whose centres sit
  // `offset` in front of and behind the body centre along the heading. The body is "clear" when BOTH centres are clear, so the
  // distance map is read twice - no rotated rectangle. NaN when either centre is outside the map. offset 0 = the old single disc.
  double footprint_clearance(double x, double y, double yaw, double offset) const {
    const double c = std::cos(yaw) * offset, s = std::sin(yaw) * offset;
    const double a = clearance(x + c, y + s), b = clearance(x - c, y - s);
    if (std::isnan(a) || std::isnan(b)) return std::numeric_limits<double>::quiet_NaN();
    return std::min(a, b);
  }
  // Direction away from the nearest blocked cell at (x, y): gradient of the distance map by central differences over +-1 cell
  // (unit length, or zero where the map is flat / outside). Used by the path smoother to push points off walls.
  Vec2 clearance_gradient(double x, double y) const {
    const double h = grid.res;
    const double ax = clearance(x + h, y), bx = clearance(x - h, y), ay = clearance(x, y + h), by = clearance(x, y - h);
    if (std::isnan(ax) || std::isnan(bx) || std::isnan(ay) || std::isnan(by)) return {0, 0};
    const double gx = ax - bx, gy = ay - by, n = std::sqrt(gx * gx + gy * gy);
    return n < 1e-9 ? Vec2{0, 0} : Vec2{gx / n, gy / n};
  }
  // Distance from (x, y) to the nearest unconfirmed prior wall; 1e6 when there is none or outside the map.
  double prior_clearance(double x, double y) const {
    int i, j;
    grid.cell(x, y, i, j);
    if (!grid.inside(i, j) || prior_dist.size() != grid.data.size()) return 1e6;
    return prior_dist[grid.idx(i, j)];
  }
};

// extra_pts: obstacle points in the map frame (fresh scan returns), marked as blocked cells before the transform.
inline CostMap build_costmap(const Grid& g, const std::vector<Vec2>& extra_pts = {}) {
  CostMap cm;
  cm.grid = g;
  std::vector<uint8_t> blocked(g.data.size()), prior(g.data.size());
  bool any_prior = false;
  for (size_t i = 0; i < g.data.size(); ++i) {
    blocked[i] = g.data[i] >= Grid::BLOCKED;
    prior[i] = Grid::is_prior(g.data[i]);
    any_prior = any_prior || prior[i];
  }
  for (const Vec2& p : extra_pts) {
    int i, j;
    g.cell(p.x, p.y, i, j);
    if (g.inside(i, j)) blocked[g.idx(i, j)] = 1;
  }
  cm.dist = distance_m(blocked, g.w, g.h, g.res);
  cm.prior_dist = any_prior ? distance_m(prior, g.w, g.h, g.res) : std::vector<float>(g.data.size(), 1e6f);
  return cm;
}

}  // namespace go2_nav
