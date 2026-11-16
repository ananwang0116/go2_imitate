// Global planner: A* on a coarse (factor x factor) copy of the costmap, then line-of-sight shortcuts and a uniform resample.
// All distances come from the geometry (spec 3). The body is two discs (2026-10-30): radius hard_m, centres fp_offset_m = half length -
// half width in front of and behind the body centre along the heading of each step (see Footprint in costmap.hpp). With that, a body centre may not come closer to a wall than
//   hard_m = body_half_width + safe_stop_gap   (safe_gate would stop the dog there anyway),
// and is pushed away by a soft cost up to soft_end_m = body_half_width + waypoint_clearance.
// Unknown cells are passable at a small extra cost (evidence first, P4). Walls of the saved map that nobody has looked at in
// this run (types.hpp PRIOR) are NOT blocked - they may have been things that stood there while mapping - but the path keeps
// the same soft distance from them as from real walls, and walking within hard_m of one costs prior_wall_penalty extra per
// metre: a wall crossing (~0.8 m) costs about 20 m of detour, so a door or another way round wins whenever there is one; with
// no way round the dog goes to look and the laser decides (a confirmed obstacle becomes BLOCKED, seen floor erases the wall).
#pragma once
#include <limits>
#include <queue>

#include "go2_nav/config.hpp"
#include "go2_nav/costmap.hpp"

namespace go2_nav {

struct PlanParams {
  double hw = 0.20;                // body_half_width_m
  double hard_m = 0.30;
  double fp_offset_m = 0.0;        // two-point footprint: centres this far in front of / behind the body centre (0 = one disc)
  double soft_end_m = 0.50;
  double soft_weight = 6.0;        // extra cost factor at the hard limit (cost = 1 + soft_weight * s^2)
  double unknown_penalty = 0.5;
  double prior_weight = 1.0 / 20.0;   // other map values 1..49: factor + value * prior_weight
  double prior_wall_penalty = 25.0;   // extra factor within hard_m of an unconfirmed prior wall
  double start_relax_m = 0.6;      // near the start the hard limit is lowered to what the dog already has
  double goal_snap_m = 0.6;        // a goal inside a wall moves to the nearest passable cell within this radius
  double shortcut_slack = 1.08;
  double resample_m = 0.1;
  int factor = 2;

  static PlanParams from_geometry(const Geometry& g) {
    PlanParams p;
    p.hw = g.body_half_width_m;
    p.hard_m = g.body_half_width_m + g.safe_stop_gap_m;
    p.soft_end_m = g.body_half_width_m + g.waypoint_clearance_m;
    // hard radius = half width + safe_stop_gap, so the two discs reach half_length + safe_stop_gap in front and behind:
    // offset = (half_length + safe_stop_gap) - hard radius = half_length - half_width. No new geometry key (spec 3).
    p.fp_offset_m = std::max(0.0, g.body_half_length_m - g.body_half_width_m);
    return p;
  }
};

struct PlanResult {
  bool ok = false;
  std::string reason;              // start_blocked | goal_blocked | no_path | no_map
  std::vector<Vec2> path;          // map frame, uniform spacing, first = start, last = (snapped) goal
  double length_m = 0.0;
  int expanded = 0;
};

inline double path_length(const std::vector<Vec2>& p) {
  double s = 0;
  for (size_t i = 1; i < p.size(); ++i) s += hyp(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
  return s;
}

class Planner {
 public:
  Planner(const CostMap& cm, const PlanParams& pp) : cm_(cm), pp_(pp) {}

  // snap_m < 0: use goal_snap_m. A waypoint passes its own radius: any passable cell inside the area is a fine place to end.
  PlanResult plan(Vec2 start, Vec2 goal, double snap_m = -1.0) {
    PlanResult r;
    snap_m_ = snap_m >= 0 ? snap_m : pp_.goal_snap_m;
    goal_ = goal;
    if (!cm_.valid()) {
      r.reason = "no_map";
      return r;
    }
    const Grid& g = cm_.grid;
    int si, sj, gi, gj;
    g.cell(start.x, start.y, si, sj);
    g.cell(goal.x, goal.y, gi, gj);
    if (!g.inside(si, sj)) {
      r.reason = "start_blocked";
      return r;
    }
    start_clear_ = cm_.dist[g.idx(si, sj)];
    start_ = start;
    relaxed_ = std::max(pp_.hw, std::min(pp_.hard_m, start_clear_));   // near the start: no stricter than what the dog already has
    if (!g.inside(gi, gj)) {
      r.reason = "goal_blocked";
      return r;
    }
    return search(start, goal, r);
  }

  // Is there a passable cell within `radius` of p (the usable end of a waypoint area)? out = the nearest one (fine grid).
  bool usable_near(Vec2 p, double radius, Vec2& out) const {
    const Grid& g = cm_.grid;
    int ci, cj;
    g.cell(p.x, p.y, ci, cj);
    const int r = static_cast<int>(std::ceil(radius / g.res));
    double best = std::numeric_limits<double>::infinity();
    for (int j = cj - r; j <= cj + r; ++j)
      for (int i = ci - r; i <= ci + r; ++i) {
        if (!g.inside(i, j) || cm_.dist[g.idx(i, j)] < pp_.hard_m) continue;
        const Vec2 c = g.center(i, j);
        const double d = hyp(c.x - p.x, c.y - p.y);
        if (d <= radius && d < best) {
          best = d;
          out = c;
        }
      }
    return std::isfinite(best);
  }

  // Cost of walking from a to b (sampled every half cell on the FINE grid); inf when a sample is not passable.
  double segment_cost(Vec2 a, Vec2 b) const {
    const double len = hyp(b.x - a.x, b.y - a.y);
    const int n = std::max(1, static_cast<int>(std::ceil(len / (cm_.grid.res * 0.5))));
    double total = 0;
    for (int k = 0; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      const Vec2 p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
      const double d = dist_at(p);
      if (d < thr_at(p)) return std::numeric_limits<double>::infinity();
      if (len > 1e-9 && oriented_at(p) && !footprint_ok(p, std::atan2(b.y - a.y, b.x - a.x))) return std::numeric_limits<double>::infinity();
      if (k > 0) total += (len / n) * cell_factor(d, cm_.prior_clearance(p.x, p.y), value_at(p));
    }
    return total;
  }

 private:
  const CostMap& cm_;
  PlanParams pp_;
  Vec2 start_{}, goal_{1e9, 1e9};
  double start_clear_ = 1e6, relaxed_ = 0.3, snap_m_ = 0.6;

  double dist_at(Vec2 p) const {
    int i, j;
    cm_.grid.cell(p.x, p.y, i, j);
    return cm_.grid.inside(i, j) ? cm_.dist[cm_.grid.idx(i, j)] : -1.0;
  }
  int value_at(Vec2 p) const {
    int i, j;
    cm_.grid.cell(p.x, p.y, i, j);
    return cm_.grid.inside(i, j) ? cm_.grid.data[cm_.grid.idx(i, j)] : 100;
  }
  double thr_at(Vec2 p) const {
    return hyp(p.x - start_.x, p.y - start_.y) <= pp_.start_relax_m ? relaxed_ : pp_.hard_m;
  }
  // Is the heading-aware footprint test in force at p? Not near the start (the dog's own clearance rules there, relaxed_) and not
  // inside the goal area (the dog can turn to face away from a wall once it is there).
  bool oriented_at(Vec2 p) const {
    return pp_.fp_offset_m > 0 && hyp(p.x - start_.x, p.y - start_.y) > pp_.start_relax_m && hyp(p.x - goal_.x, p.y - goal_.y) > snap_m_;
  }
  // Both footprint centres at p heading `yaw` keep thr_at(p) from the blocked cells (fine grid, exact).
  bool footprint_ok(Vec2 p, double yaw) const {
    const double c = std::cos(yaw) * pp_.fp_offset_m, s = std::sin(yaw) * pp_.fp_offset_m;
    const double thr = thr_at(p);
    return dist_at({p.x + c, p.y + s}) >= thr && dist_at({p.x - c, p.y - s}) >= thr;
  }
  // multiplier of the walking cost in a cell with clearance d (blocked cells), pd (unconfirmed prior walls) and map value v
  double cell_factor(double d, double pd, int v) const {
    const double near = std::min(d, pd);
    double s = clampd((pp_.soft_end_m - near) / std::max(1e-6, pp_.soft_end_m - pp_.hard_m), 0.0, 1.0);
    double f = 1.0 + pp_.soft_weight * s * s;
    if (pd < pp_.hard_m) f += pp_.prior_wall_penalty;
    if (v < 0) f += pp_.unknown_penalty;
    else if (v > 0 && !Grid::is_prior(static_cast<int8_t>(v))) f += pp_.prior_weight * v;
    return f;
  }

  PlanResult search(Vec2 start, Vec2 goal, PlanResult r) {
    const Grid& g = cm_.grid;
    const int f = std::max(1, pp_.factor);
    const int cw = (g.w + f - 1) / f, ch = (g.h + f - 1) / f;
    const double cres = g.res * f;
    // coarse view: conservative clearance (min of the children), worst map value, unknown only if all children are
    std::vector<float> cd(static_cast<size_t>(cw) * ch, 1e6f), cpd(cd.size(), 1e6f);
    std::vector<int8_t> cv(cd.size(), -1);
    const bool have_prior = cm_.prior_dist.size() == g.data.size();
    for (int j = 0; j < g.h; ++j)
      for (int i = 0; i < g.w; ++i) {
        const size_t k = static_cast<size_t>(j / f) * cw + (i / f), s = g.idx(i, j);
        cd[k] = std::min(cd[k], cm_.dist[s]);
        if (have_prior) cpd[k] = std::min(cpd[k], cm_.prior_dist[s]);
        const int8_t v = g.data[s];
        if (v >= 0 && (cv[k] < 0 || v > cv[k])) cv[k] = v;
      }
    auto pos = [&](int I, int J) { return Vec2{g.ox + (I + 0.5) * cres, g.oy + (J + 0.5) * cres}; };
    auto passable = [&](int I, int J) {
      const size_t k = static_cast<size_t>(J) * cw + I;
      return cd[k] >= thr_at(pos(I, J)) - 0.5 * g.res * f;   // min of the children, judged against the limit minus half a coarse cell
    };
    // coarse cell (I, J) entered heading (di, dj): its centre must pass, and so must both footprint centres (conservative coarse
    // clearance = min of the children, judged against the limit minus half a coarse cell, like `passable`)
    auto passable_dir = [&](int I, int J, int di, int dj) {
      if (!passable(I, J)) return false;
      const Vec2 c = pos(I, J);
      if (!oriented_at(c)) return true;
      const double yaw = std::atan2(static_cast<double>(dj), static_cast<double>(di));
      const double ox = std::cos(yaw) * pp_.fp_offset_m, oy = std::sin(yaw) * pp_.fp_offset_m;
      const double thr = thr_at(c) - 0.5 * g.res * f;
      for (double sgn : {1.0, -1.0}) {
        const int PI = static_cast<int>(std::floor((c.x + sgn * ox - g.ox) / cres)), PJ = static_cast<int>(std::floor((c.y + sgn * oy - g.oy) / cres));
        if (PI < 0 || PJ < 0 || PI >= cw || PJ >= ch) return false;                     // outside the map = blocked
        if (cd[static_cast<size_t>(PJ) * cw + PI] < thr) return false;
      }
      return true;
    };
    int sI = std::min(cw - 1, std::max(0, static_cast<int>(std::floor((start.x - g.ox) / cres))));
    int sJ = std::min(ch - 1, std::max(0, static_cast<int>(std::floor((start.y - g.oy) / cres))));
    int gI = std::min(cw - 1, std::max(0, static_cast<int>(std::floor((goal.x - g.ox) / cres))));
    int gJ = std::min(ch - 1, std::max(0, static_cast<int>(std::floor((goal.y - g.oy) / cres))));
    // a start / goal cell that is not passable moves to the nearest passable coarse cell (goal: goal_snap_m, start: start_relax_m)
    auto snap = [&](int& I, int& J, double radius) {
      if (passable(I, J)) return true;
      const int R = static_cast<int>(std::ceil(radius / cres));
      double best = 1e18;
      int bi = -1, bj = -1;
      for (int dj = -R; dj <= R; ++dj)
        for (int di = -R; di <= R; ++di) {
          const int a = I + di, b = J + dj;
          if (a < 0 || b < 0 || a >= cw || b >= ch || !passable(a, b)) continue;
          const double d = cres * hyp(di, dj);
          if (d <= radius && d < best) {
            best = d;
            bi = a;
            bj = b;
          }
        }
      if (bi < 0) return false;
      I = bi;
      J = bj;
      return true;
    };
    bool start_snapped = false, goal_snapped = false;
    {
      const int oi = sI, oj = sJ;
      if (!snap(sI, sJ, pp_.start_relax_m)) {
        r.reason = "start_blocked";
        return r;
      }
      start_snapped = sI != oi || sJ != oj;
    }
    {
      const int oi = gI, oj = gJ;
      if (!snap(gI, gJ, snap_m_)) {
        r.reason = "goal_blocked";
        return r;
      }
      goal_snapped = gI != oi || gJ != oj;
    }
    const size_t S = static_cast<size_t>(sJ) * cw + sI, G = static_cast<size_t>(gJ) * cw + gI;

    const float INF = std::numeric_limits<float>::infinity();
    std::vector<float> gc(cd.size(), INF);
    std::vector<int32_t> parent(cd.size(), -1);
    std::vector<uint8_t> closed(cd.size(), 0);
    using QE = std::pair<float, uint32_t>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> open;
    auto heur = [&](int I, int J) {            // octile distance, admissible (every factor >= 1)
      const double dx = std::abs(I - gI), dy = std::abs(J - gJ);
      return static_cast<float>(cres * ((dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy)));
    };
    gc[S] = 0;
    open.push({heur(sI, sJ), static_cast<uint32_t>(S)});
    static const int DI[8] = {1, -1, 0, 0, 1, 1, -1, -1}, DJ[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    bool found = false;
    while (!open.empty()) {
      const uint32_t cur = open.top().second;
      open.pop();
      if (closed[cur]) continue;
      closed[cur] = 1;
      ++r.expanded;
      if (cur == G) {
        found = true;
        break;
      }
      const int I = cur % cw, J = cur / cw;
      for (int n = 0; n < 8; ++n) {
        const int NI = I + DI[n], NJ = J + DJ[n];
        if (NI < 0 || NJ < 0 || NI >= cw || NJ >= ch) continue;
        const size_t k = static_cast<size_t>(NJ) * cw + NI;
        if (closed[k] || !passable_dir(NI, NJ, DI[n], DJ[n])) continue;
        if (n >= 4 && (!passable(I + DI[n], J) || !passable(I, J + DJ[n]))) continue;     // no corner cutting
        const double step = cres * (n >= 4 ? std::sqrt(2.0) : 1.0);
        const float ng = gc[cur] + static_cast<float>(step * cell_factor(cd[k], cpd[k], cv[k]));
        if (ng < gc[k]) {
          gc[k] = ng;
          parent[k] = static_cast<int32_t>(cur);
          open.push({ng + heur(NI, NJ), static_cast<uint32_t>(k)});
        }
      }
    }
    if (!found) {
      r.reason = "no_path";
      return r;
    }
    std::vector<Vec2> raw;
    for (int32_t c = static_cast<int32_t>(G); c >= 0; c = parent[c]) raw.push_back(pos(c % cw, c / cw));
    std::reverse(raw.begin(), raw.end());
    raw.front() = start_snapped ? pos(sI, sJ) : start;
    raw.back() = goal_snapped ? pos(gI, gJ) : goal;
    r.path = resample(shortcut(raw), pp_.resample_m);
    r.length_m = path_length(r.path);
    r.ok = true;
    return r;
  }

  // Greedy line-of-sight shortcut: jump to the farthest point whose straight segment costs no more than the original
  // route (+ slack), so the path stays away from walls where the soft cost told it to.
  std::vector<Vec2> shortcut(const std::vector<Vec2>& p) const {
    std::vector<Vec2> out{p.front()};
    size_t i = 0;
    while (i + 1 < p.size()) {
      size_t best = i + 1;
      double orig = 0;
      std::vector<double> acc(p.size(), 0.0);
      const size_t last = std::min(p.size() - 1, i + 80);
      for (size_t j = i + 1; j <= last; ++j) {
        acc[j] = acc[j - 1 > i ? j - 1 : i] + segment_cost(p[j - 1], p[j]);
      }
      for (size_t j = last; j > i + 1; --j) {
        orig = acc[j];
        const double direct = segment_cost(p[i], p[j]);
        if (std::isfinite(direct) && direct <= orig * pp_.shortcut_slack + 1e-6) {
          best = j;
          break;
        }
      }
      out.push_back(p[best]);
      i = best;
    }
    return out;
  }

  static std::vector<Vec2> resample(const std::vector<Vec2>& p, double step) {
    std::vector<Vec2> out{p.front()};
    double carry = 0;
    for (size_t i = 1; i < p.size(); ++i) {
      const double len = hyp(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
      double pos = step - carry;
      while (pos <= len) {
        const double t = pos / len;
        out.push_back({p[i - 1].x + (p[i].x - p[i - 1].x) * t, p[i - 1].y + (p[i].y - p[i - 1].y) * t});
        pos += step;
      }
      carry = len - (pos - step);
    }
    if (hyp(out.back().x - p.back().x, out.back().y - p.back().y) > 1e-6) out.push_back(p.back());
    return out;
  }
};

}  // namespace go2_nav
