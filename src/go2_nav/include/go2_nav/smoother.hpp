// Path smoother (SCAN-Planner's B-spline back end, docs/05 section 9, item 6), 2D and geometric.
// Takes the A* path (a polyline with corners) and returns a smooth curve:
//   - uniform cubic B-spline whose control points start on the input path every `ctrl_m`; the first / last few are fixed, so the
//     curve starts at the dog and ends at the goal, and a relayed (kept) head stays where it was;
//   - cost  w_smooth * jerk of the control points   (SCAN-Planner's smoothness term)
//         + w_fit * distance of each control point from where A* put it   (stay in the same corridor / on the same side)
//         + w_clear * shortfall of the clearance of both footprint centres on curve samples ("push off"): a sample closer to a
//           wall than `want` is moved away from it along the gradient of the distance map. `want` is the hard radius + extra but
//           never more than the input path itself had at that sample, so a squeezed spot is only kept, never made impossible.
//   - plain gradient descent with backtracking (the problem has ~100 variables).
// Differences from SCAN-Planner, on purpose: no time parametrisation, so no velocity / acceleration limits here (the follower's
// rate limiter keeps those); the push direction comes from our distance map, not from a second A* search.
// SAFE BY CONSTRUCTION: the result is accepted only if its worst footprint clearance (outside the dog's start zone and the goal
// area) is not below min(hard radius - 2 cm, the input path's own worst); otherwise ok = false and the caller keeps the A* path.
#pragma once
#include "go2_nav/planner.hpp"

namespace go2_nav {

struct SmoothParams {
  double ctrl_m = 0.2;         // control point spacing (SCAN-Planner: 0.2)
  double sample_m = 0.1;       // spacing of the output path
  double w_smooth = 1.0, w_fit = 1.0, w_clear = 60.0;
  double extra_m = 0.05;       // wanted clearance = hard radius + this
  double max_move_m = 0.6;     // a control point never moves farther than this from its A* place
  int iters = 120;
  double fix_head_m = 0.0;     // keep the first this many metres as they are (a relayed head); at least 3 control points are fixed
  int tail_fixed = 3;
  double skip_head_m = 0.6;    // acceptance and clearance ignore the start zone ...
  double skip_tail_m = 0.4;    // ... and the goal area (the planner does not enforce the heading test there either)
  double max_stretch = 1.3;    // reject a curve longer than this x the input + 0.5 m
};

struct SmoothResult {
  bool ok = false;
  std::string why;             // when not ok: short | clearance | stretch | outside
  std::vector<Vec2> path;
  int iters = 0;
  double in_min_m = 0, out_min_m = 0;   // worst footprint clearance of the input / of the curve (outside the skip zones)
};

namespace smooth_detail {

inline void basis(double u, double b[4]) {
  const double u2 = u * u, u3 = u2 * u;
  b[0] = (1 - u) * (1 - u) * (1 - u) / 6.0;
  b[1] = (3 * u3 - 6 * u2 + 4) / 6.0;
  b[2] = (-3 * u3 + 3 * u2 + 3 * u + 1) / 6.0;
  b[3] = u3 / 6.0;
}
inline void dbasis(double u, double b[4]) {         // d/du
  const double u2 = u * u;
  b[0] = -(1 - u) * (1 - u) / 2.0;
  b[1] = (9 * u2 - 12 * u) / 6.0;
  b[2] = (-9 * u2 + 6 * u + 3) / 6.0;
  b[3] = u2 / 2.0;
}

// control points of the padded spline: [Q0, Q0, Q..., Qn, Qn] so that the curve starts at Q0 and ends at Qn
inline std::vector<Vec2> pad(const std::vector<Vec2>& q) {
  std::vector<Vec2> c;
  c.reserve(q.size() + 4);
  c.push_back(q.front());
  c.push_back(q.front());
  c.insert(c.end(), q.begin(), q.end());
  c.push_back(q.back());
  c.push_back(q.back());
  return c;
}

inline std::vector<Vec2> resample_arc(const std::vector<Vec2>& p, double step) {
  std::vector<Vec2> out{p.front()};
  double carry = 0;
  for (size_t i = 1; i < p.size(); ++i) {
    const double len = hyp(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
    if (len < 1e-12) continue;
    double pos = step - carry;
    while (pos <= len + 1e-12) {
      const double t = pos / len;
      out.push_back({p[i - 1].x + (p[i].x - p[i - 1].x) * t, p[i - 1].y + (p[i].y - p[i - 1].y) * t});
      pos += step;
    }
    carry = len - (pos - step);
  }
  if (hyp(out.back().x - p.back().x, out.back().y - p.back().y) > 1e-6) out.push_back(p.back());
  return out;
}

// worst footprint clearance along a polyline, skipping `head` / `tail` metres of arc at the ends; NaN if a centre leaves the map
inline double worst_clearance(const CostMap& cm, double offset, const std::vector<Vec2>& poly, double head, double tail) {
  double total = 0;
  for (size_t i = 1; i < poly.size(); ++i) total += hyp(poly[i].x - poly[i - 1].x, poly[i].y - poly[i - 1].y);
  double s = 0, worst = 1e9;
  for (size_t i = 0; i + 1 < poly.size(); ++i) {
    const double len = hyp(poly[i + 1].x - poly[i].x, poly[i + 1].y - poly[i].y);
    if (len > 1e-9 && s >= head && s <= total - tail) {
      const double d = cm.footprint_clearance(poly[i].x, poly[i].y, std::atan2(poly[i + 1].y - poly[i].y, poly[i + 1].x - poly[i].x), offset);
      if (std::isnan(d)) return d;
      worst = std::min(worst, d);
    }
    s += len;
  }
  return worst;
}

}  // namespace smooth_detail

inline SmoothResult smooth_path(const CostMap& cm, const PlanParams& pp, const SmoothParams& sp, const std::vector<Vec2>& in) {
  using namespace smooth_detail;
  SmoothResult res;
  if (!cm.valid() || in.size() < 3) {
    res.why = "short";
    return res;
  }
  const double off = pp.fp_offset_m;
  // control points on the input path every ctrl_m, the last one is the goal
  const std::vector<Vec2> dense = resample_arc(in, 0.02);
  std::vector<Vec2> Q0;
  {
    double acc = 0, next = 0;
    Q0.push_back(dense.front());
    next = sp.ctrl_m;
    for (size_t i = 1; i < dense.size(); ++i) {
      acc += hyp(dense[i].x - dense[i - 1].x, dense[i].y - dense[i - 1].y);
      if (acc >= next - 1e-9) {
        Q0.push_back(dense[i]);
        next += sp.ctrl_m;
      }
    }
    if (hyp(Q0.back().x - dense.back().x, Q0.back().y - dense.back().y) > 0.5 * sp.ctrl_m) Q0.push_back(dense.back());
    else Q0.back() = dense.back();
  }
  const int n = static_cast<int>(Q0.size());
  const int head_fixed = std::max(3, static_cast<int>(std::ceil(sp.fix_head_m / sp.ctrl_m)) + 1);
  if (n < head_fixed + sp.tail_fixed + 1) {
    res.why = "short";
    return res;
  }
  std::vector<char> fixed(n, 0);
  for (int i = 0; i < n; ++i) fixed[i] = i < head_fixed || i >= n - sp.tail_fixed;
  const int nseg = n + 4 - 3;                          // segments of the padded spline
  const int m = std::max(2, static_cast<int>(std::ceil(sp.ctrl_m / 0.05)));      // samples per segment for the clearance term

  // the wanted clearance per sample and per footprint centre: hard + extra, but not more than the input curve had there
  const double want_max = pp.hard_m + sp.extra_m;
  struct Samp { int seg; double u; double want_f, want_r; };
  std::vector<Samp> samples;
  {
    const std::vector<Vec2> c0 = pad(Q0);
    for (int s = 0; s < nseg; ++s)
      for (int k = 0; k < m; ++k) {
        const double u = static_cast<double>(k) / m;
        double b[4], d[4];
        basis(u, b);
        dbasis(u, d);
        Vec2 p{0, 0}, t{0, 0};
        for (int j = 0; j < 4; ++j) {
          p.x += b[j] * c0[s + j].x;
          p.y += b[j] * c0[s + j].y;
          t.x += d[j] * c0[s + j].x;
          t.y += d[j] * c0[s + j].y;
        }
        const double yaw = std::atan2(t.y, t.x), cx = std::cos(yaw) * off, cy = std::sin(yaw) * off;
        const double df = cm.clearance(p.x + cx, p.y + cy), dr = cm.clearance(p.x - cx, p.y - cy);
        samples.push_back({s, u, std::isnan(df) ? 0.0 : std::min(want_max, df), std::isnan(dr) ? 0.0 : std::min(want_max, dr)});
      }
  }

  // cost and gradient (on the padded control points, then folded back onto Q)
  auto eval = [&](const std::vector<Vec2>& Q, std::vector<Vec2>& g) {
    const std::vector<Vec2> C = pad(Q);
    std::vector<Vec2> gc(C.size(), Vec2{0, 0});
    double cost = 0;
    for (size_t i = 0; i + 3 < C.size(); ++i) {                         // jerk of the control points
      const Vec2 j{C[i + 3].x - 3 * C[i + 2].x + 3 * C[i + 1].x - C[i].x, C[i + 3].y - 3 * C[i + 2].y + 3 * C[i + 1].y - C[i].y};
      cost += sp.w_smooth * (j.x * j.x + j.y * j.y);
      const double gx = 2 * sp.w_smooth * j.x, gy = 2 * sp.w_smooth * j.y;
      gc[i].x -= gx; gc[i].y -= gy;
      gc[i + 1].x += 3 * gx; gc[i + 1].y += 3 * gy;
      gc[i + 2].x -= 3 * gx; gc[i + 2].y -= 3 * gy;
      gc[i + 3].x += gx; gc[i + 3].y += gy;
    }
    for (const Samp& sm : samples) {                                    // push off walls
      double b[4], d[4];
      basis(sm.u, b);
      dbasis(sm.u, d);
      Vec2 p{0, 0}, t{0, 0};
      for (int j = 0; j < 4; ++j) {
        p.x += b[j] * C[sm.seg + j].x;
        p.y += b[j] * C[sm.seg + j].y;
        t.x += d[j] * C[sm.seg + j].x;
        t.y += d[j] * C[sm.seg + j].y;
      }
      const double yaw = std::atan2(t.y, t.x), cx = std::cos(yaw) * off, cy = std::sin(yaw) * off;
      for (int side = 0; side < 2; ++side) {
        const double sgn = side == 0 ? 1.0 : -1.0, want = side == 0 ? sm.want_f : sm.want_r;
        const double ex = p.x + sgn * cx, ey = p.y + sgn * cy;
        const double dist = cm.clearance(ex, ey);
        if (std::isnan(dist) || dist >= want) continue;
        const double sh = want - dist;
        cost += sp.w_clear * sh * sh;
        const Vec2 gr = cm.clearance_gradient(ex, ey);
        const double f = -2 * sp.w_clear * sh;                          // d cost / d dist
        for (int j = 0; j < 4; ++j) {
          gc[sm.seg + j].x += f * gr.x * b[j];
          gc[sm.seg + j].y += f * gr.y * b[j];
        }
      }
    }
    g.assign(Q.size(), Vec2{0, 0});
    for (size_t i = 0; i < Q.size(); ++i) {                             // fold the padded gradient back; fit term
      g[i] = gc[i + 2];
      const double fx = Q[i].x - Q0[i].x, fy = Q[i].y - Q0[i].y;
      cost += sp.w_fit * (fx * fx + fy * fy);
      g[i].x += 2 * sp.w_fit * fx;
      g[i].y += 2 * sp.w_fit * fy;
    }
    g[0].x += gc[0].x + gc[1].x;  g[0].y += gc[0].y + gc[1].y;          // (fixed anyway)
    g[n - 1].x += gc[n + 2].x + gc[n + 3].x;  g[n - 1].y += gc[n + 2].y + gc[n + 3].y;
    for (int i = 0; i < n; ++i)
      if (fixed[i]) g[i] = {0, 0};
    return cost;
  };

  std::vector<Vec2> Q = Q0, g, Qn;
  double f = eval(Q, g), alpha = 0.05;
  int it = 0;
  for (; it < sp.iters; ++it) {
    double gn2 = 0;
    for (const Vec2& v : g) gn2 += v.x * v.x + v.y * v.y;
    if (gn2 < 1e-10) break;
    bool moved = false;
    for (int tries = 0; tries < 25; ++tries) {
      Qn = Q;
      for (int i = 0; i < n; ++i) {
        if (fixed[i]) continue;
        Qn[i].x = Q[i].x - alpha * g[i].x;
        Qn[i].y = Q[i].y - alpha * g[i].y;
        const double dx = Qn[i].x - Q0[i].x, dy = Qn[i].y - Q0[i].y, dd = std::sqrt(dx * dx + dy * dy);
        if (dd > sp.max_move_m) {
          Qn[i].x = Q0[i].x + dx / dd * sp.max_move_m;
          Qn[i].y = Q0[i].y + dy / dd * sp.max_move_m;
        }
      }
      std::vector<Vec2> gnew;
      const double fnew = eval(Qn, gnew);
      if (fnew <= f - 1e-4 * alpha * gn2) {
        Q = Qn;
        g = gnew;
        f = fnew;
        alpha = std::min(alpha * 1.6, 1.0);
        moved = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!moved) break;
  }
  res.iters = it;

  // the curve, finely, then at sample_m
  const std::vector<Vec2> C = pad(Q);
  std::vector<Vec2> fine;
  const int mf = std::max(2, static_cast<int>(std::ceil(sp.ctrl_m / 0.02)));
  for (int s = 0; s < nseg; ++s)
    for (int k = 0; k < mf; ++k) {
      double b[4];
      basis(static_cast<double>(k) / mf, b);
      Vec2 p{0, 0};
      for (int j = 0; j < 4; ++j) {
        p.x += b[j] * C[s + j].x;
        p.y += b[j] * C[s + j].y;
      }
      fine.push_back(p);
    }
  fine.push_back(Q.back());
  std::vector<Vec2> out = resample_arc(fine, sp.sample_m);

  // accept only if it is not worse than the input
  const std::vector<Vec2> in10 = resample_arc(in, sp.sample_m);
  res.in_min_m = worst_clearance(cm, off, in10, sp.skip_head_m, sp.skip_tail_m);
  res.out_min_m = worst_clearance(cm, off, out, sp.skip_head_m, sp.skip_tail_m);
  if (std::isnan(res.out_min_m) || std::isnan(res.in_min_m)) {
    res.why = "outside";
    return res;
  }
  const double floor_m = std::min(pp.hard_m - 0.02, res.in_min_m);
  if (res.out_min_m < floor_m - 1e-6) {
    res.why = "clearance";
    return res;
  }
  if (path_length(out) > sp.max_stretch * path_length(in) + 0.5) {
    res.why = "stretch";
    return res;
  }
  res.path = std::move(out);
  res.ok = true;
  return res;
}

}  // namespace go2_nav
