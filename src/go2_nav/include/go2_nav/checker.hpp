// Reachability checks asked of the planner that drives the dog (service /mot_nav_check): one judge for "can it get there" (P2).
#pragma once
#include "go2_nav/pathmode.hpp"

namespace go2_nav {

struct CheckIn {
  std::vector<Vec2> pos;
  std::vector<double> radius;
  std::vector<std::vector<Vec2>> paths;     // per waypoint, may be empty
  bool use_start = false;
  Vec2 start;
};
struct CheckOut {
  std::vector<bool> ok;
  std::vector<std::string> reason;
  std::vector<double> leg_m;
  std::vector<std::vector<Vec2>> leg_path;    // the A* path of each leg (empty where leg_m is 0), for display
};

inline CheckOut check_waypoints(const CostMap& cm, const PlanParams& pp, const Geometry& g, const CheckIn& in) {
  CheckOut out;
  const Grid& grid = cm.grid;
  Planner planner(cm, pp);
  bool have_prev = in.use_start;
  Vec2 prev = in.start;
  for (size_t k = 0; k < in.pos.size(); ++k) {
    std::string why;
    double leg = 0;
    std::vector<Vec2> leg_path;
    const Vec2 w = in.pos[k];
    const double radius = k < in.radius.size() ? in.radius[k] : g.arrive_radius_m;
    int i, j;
    grid.cell(w.x, w.y, i, j);
    Vec2 usable{w.x, w.y};
    if (!grid.inside(i, j)) {
      why = "outside the map";
    } else {
      if (radius + 1e-9 < g.loc_sigma_m) why = "radius_m " + fmt_num(radius) + " < loc_sigma_m " + fmt_num(g.loc_sigma_m);
      // share of unknown cells in the area
      int total = 0, unknown = 0;
      const int r = static_cast<int>(std::ceil(radius / grid.res));
      for (int jj = j - r; jj <= j + r; ++jj)
        for (int ii = i - r; ii <= i + r; ++ii) {
          if (!grid.inside(ii, jj)) continue;
          const Vec2 c = grid.center(ii, jj);
          if (hyp(c.x - w.x, c.y - w.y) > radius) continue;
          ++total;
          if (grid.data[grid.idx(ii, jj)] < 0) ++unknown;
        }
      if (why.empty() && total > 0 && unknown > 0.5 * total) why = "area mostly unknown on the map";
      if (why.empty() && !planner.usable_near(w, radius, usable)) why = "no passable position in the area (hard clearance " + fmt_num(pp.hard_m) + " m)";
      if (why.empty() && k < in.paths.size() && in.paths[k].size() > 1) {      // a taught path must keep the hard clearance
        double since = 1e9;
        const auto& p = in.paths[k];
        for (size_t q = 0; q < p.size() && why.empty(); ++q) {
          if (q > 0) since += hyp(p[q].x - p[q - 1].x, p[q].y - p[q - 1].y);
          if (since < 0.5 && q + 1 < p.size()) continue;
          since = 0;
          if (cm.clearance(p[q].x, p[q].y) < pp.hard_m * 0.9) why = "taught path blocked near (" + fmt_num(p[q].x) + ", " + fmt_num(p[q].y) + ")";
        }
      }
      if (why.empty() && have_prev) {
        const PlanResult pr = planner.plan(prev, w, radius);
        if (!pr.ok) why = std::string("not connected to ") + (k == 0 ? "the dog" : "the previous waypoint") + " (" + pr.reason + ")";
        else {
          leg = pr.length_m;
          leg_path = pr.path;
          usable = pr.path.back();
        }
      }
    }
    out.ok.push_back(why.empty());
    out.reason.push_back(why);
    out.leg_m.push_back(leg);
    out.leg_path.push_back(leg_path);
    if (why.empty()) {
      prev = usable;
      have_prev = true;
    }
  }
  return out;
}

}  // namespace go2_nav
