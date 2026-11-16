// Local follower: pure pursuit toward a look-ahead point on the path + a small trajectory sampler for avoidance
// (a simplified MPPI / DWA): candidate (vx, vy, wz) are rolled out for horizon_s in the body frame, rejected when the body
// (inflated by safe_stop_gap) would touch a fresh scan point or a BLOCKED map cell (types.hpp), and the cheapest one is the command.
// Scan points are tested against the real rectangle; the MAP walls against the two-disc footprint (costmap.hpp). safe_gate still limits
// everything afterwards (it is not told about this node). No ROS in here.
#pragma once
#include "go2_nav/config.hpp"
#include "go2_nav/costmap.hpp"

namespace go2_nav {

struct FollowParams {
  double hl = 0.35, hw = 0.20;         // body half length / width
  double fp_offset = 0.15;             // two-point footprint for the MAP walls: centres hl - hw in front of / behind the body centre
  double margin = 0.10;                // safe_stop_gap_m: rollouts closer than this to a point are collisions
  double avoid_gap = 0.20;             // avoid_gap_m: clearance reward ends here
  double soft_end = 0.50;              // body_half_width + waypoint_clearance: slow down inside
  double vf = 0.5, vb = 0.3, vs = 0.15, wmax = 0.5;     // speeds.yaml (side capped at 0.15)
  double min_move = 0.15;              // the dog does not walk below ~0.13 m/s (safe_gate cuts < 0.12)
  double lookahead_min = 0.6, lookahead_gain = 0.8, lookahead_max = 1.0;
  double horizon_s = 1.5, dt = 0.1;
  double slow_dist = 1.0;              // slow down over the last metre
  double w_goal = 1.0, w_head = 2.0, w_path = 1.5, w_clear = 1.0, w_smooth = 0.1, w_speed = 0.6;
  double turn_first_rad = 1.2;         // bearing error beyond this: turn in place first
  double accel = 1.0, ang_accel = 2.0; // command rate limit [m/s^2, rad/s^2]

  static FollowParams from(const Geometry& g, const Speeds& s) {
    FollowParams p;
    p.hl = g.body_half_length_m;
    p.hw = g.body_half_width_m;
    p.fp_offset = std::max(0.0, g.body_half_length_m - g.body_half_width_m);
    p.margin = g.safe_stop_gap_m;
    p.avoid_gap = g.avoid_gap_m;
    p.soft_end = g.body_half_width_m + g.waypoint_clearance_m;
    p.vf = s.forward_mps;
    p.vb = s.back_mps;
    p.vs = std::min(0.15, s.side_mps);
    p.wmax = s.turn_radps;
    return p;
  }
};

struct FollowOut {
  Cmd cmd;
  bool blocked = false;                // every candidate collides: the dog stops (the caller decides when this is a failure)
  double remaining_m = 0;              // along the path
  double clearance_m = std::numeric_limits<double>::quiet_NaN();   // body centre to the nearest wall (map)
  double bearing_rad = 0;
};

class Follower {
 public:
  explicit Follower(const FollowParams& p) : p_(p) {}
  void reset() { hint_ = 0; }

  // pose: map frame. path: map frame. pts: fresh obstacle points in the MAP frame. cm: may be null.
  // scale: speed_pct / 100. prev: the command sent last tick (for smoothness and the rate limiter).
  FollowOut step(const Pose2& pose, const std::vector<Vec2>& path, const std::vector<Vec2>& pts, const CostMap* cm, double scale,
                 const Cmd& prev, double tick_s) {
    FollowOut out;
    if (path.size() < 2) return out;
    // closest path point, never going back (a path that loops past the robot must not make it jump ahead: look 3 m on)
    size_t best = hint_;
    double bd = 1e18;
    for (size_t i = hint_; i < path.size() && i < hint_ + 40; ++i) {
      const double d = hyp(path[i].x - pose.x, path[i].y - pose.y);
      if (d < bd) {
        bd = d;
        best = i;
      }
    }
    hint_ = best;
    double rest = 0;
    for (size_t i = best + 1; i < path.size(); ++i) rest += hyp(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    out.remaining_m = rest + bd;
    const double L = clampd(p_.lookahead_min + p_.lookahead_gain * std::fabs(prev.vx), p_.lookahead_min, p_.lookahead_max);
    size_t ti = best;
    for (double acc = 0; ti + 1 < path.size() && acc < L; ++ti) acc += hyp(path[ti + 1].x - path[ti].x, path[ti + 1].y - path[ti].y);
    const Vec2 target = path[ti];
    const double bearing = wrap(std::atan2(target.y - pose.y, target.x - pose.x) - pose.yaw);
    out.bearing_rad = bearing;
    if (cm) out.clearance_m = cm->clearance(pose.x, pose.y);
    const double fp_now = cm ? cm->footprint_clearance(pose.x, pose.y, pose.yaw, p_.fp_offset) : std::numeric_limits<double>::quiet_NaN();

    // speed budget: slower in the last metre and in the soft zone along walls, never below what the dog can walk
    double vmax = p_.vf * scale;
    vmax *= clampd(out.remaining_m / p_.slow_dist, 0.0, 1.0);
    if (std::isfinite(out.clearance_m) && out.clearance_m < p_.soft_end) vmax *= 0.6;
    vmax = std::max(vmax, std::min(p_.min_move, p_.vf * scale));

    // body-frame obstacle points (rotate the map-frame points into the robot frame)
    const double c0 = std::cos(pose.yaw), s0 = std::sin(pose.yaw);
    std::vector<Vec2> bp;
    bp.reserve(pts.size());
    for (const Vec2& q : pts) {
      const double dx = q.x - pose.x, dy = q.y - pose.y;
      bp.push_back({c0 * dx + s0 * dy, -s0 * dx + c0 * dy});
    }
    // per point: how close it is now. A point already inside the margin may not get closer; any other point may not come inside it.
    std::vector<double> limit(bp.size());
    for (size_t i = 0; i < bp.size(); ++i) {
      const double d0 = point_dist(bp[i], 0, 0, 0);
      limit[i] = d0 >= p_.margin ? p_.margin : d0 - 0.02;
    }

    const bool turn_only = std::fabs(bearing) > p_.turn_first_rad;
    std::vector<double> vxs, vys, wzs;
    if (!turn_only) {
      for (double f : {0.4, 0.7, 1.0}) vxs.push_back(std::max(f * vmax, std::min(p_.min_move, vmax)));
    }
    vxs.push_back(0.0);
    for (double f : {-1.0, 0.0, 1.0}) vys.push_back(f * p_.vs * scale);
    for (int k = -4; k <= 4; ++k) wzs.push_back(p_.wmax * scale * k / 4.0);

    double best_score = 1e18;
    Cmd best_cmd;
    bool any = false;
    const int steps = std::max(1, static_cast<int>(p_.horizon_s / p_.dt));
    for (double vx : vxs)
      for (double vy : vys) {
        if (vx == 0.0 && vy != 0.0 && turn_only) continue;
        for (double wz : wzs) {
          if (vx == 0.0 && vy == 0.0 && wz == 0.0) continue;    // standing still is not an answer: no candidate = blocked
          double x = 0, y = 0, th = 0, clr = 1e9;
          bool hit = false;
          for (int s = 1; s <= steps && !hit; ++s) {
            x += (vx * std::cos(th) - vy * std::sin(th)) * p_.dt;
            y += (vx * std::sin(th) + vy * std::cos(th)) * p_.dt;
            th += wz * p_.dt;
            for (size_t i = 0; i < bp.size(); ++i) {
              const double c = point_dist(bp[i], x, y, th);
              clr = std::min(clr, c);
              if (c < limit[i]) {
                hit = true;
                break;
              }
            }
            if (!hit && cm) {                                               // BLOCKED map cells: confirmed obstacles and hand-drawn walls (glass the laser does not see); unconfirmed prior walls are not checked here, the fresh scan is
              const double wx = pose.x + c0 * x - s0 * y, wy = pose.y + s0 * x + c0 * y;
              const double d = cm->footprint_clearance(wx, wy, pose.yaw + th, p_.fp_offset);
              if (std::isfinite(d) && d < p_.hw * 0.95 && (!std::isfinite(fp_now) || d < fp_now - 0.01)) hit = true;
            }
          }
          if (hit) continue;
          const double wx = pose.x + c0 * x - s0 * y, wy = pose.y + s0 * x + c0 * y, wth = pose.yaw + th;
          const double to_goal = hyp(target.x - wx, target.y - wy);
          const double head = std::fabs(wrap(std::atan2(target.y - wy, target.x - wx) - wth)) / M_PI;
          double dev = 1e18;
          for (size_t i = best; i < path.size() && i < best + 60; ++i) dev = std::min(dev, hyp(path[i].x - wx, path[i].y - wy));
          const double reward = std::max(0.0, p_.avoid_gap - std::min(clr, p_.avoid_gap)) / p_.avoid_gap;
          const double score = p_.w_goal * to_goal + p_.w_head * head + p_.w_path * dev + p_.w_clear * reward +
                               p_.w_smooth * (std::fabs(vx - prev.vx) / p_.vf + std::fabs(vy - prev.vy) / p_.vs +
                                              std::fabs(wz - prev.wz) / p_.wmax) -
                               p_.w_speed * vx / p_.vf;
          if (score < best_score) {
            best_score = score;
            best_cmd = {vx, vy, wz};
            any = true;
          }
        }
      }
    if (!any) {
      out.blocked = true;
      out.cmd = {0, 0, 0};
      return out;
    }
    // rate limit so the dog is not jerked between candidates
    const double da = p_.accel * tick_s, dw = p_.ang_accel * tick_s;
    out.cmd.vx = prev.vx + clampd(best_cmd.vx - prev.vx, -da, da);
    out.cmd.vy = prev.vy + clampd(best_cmd.vy - prev.vy, -da, da);
    out.cmd.wz = prev.wz + clampd(best_cmd.wz - prev.wz, -dw, dw);
    if (std::fabs(out.cmd.vx) < 0.02 && best_cmd.vx == 0.0) out.cmd.vx = 0;
    if (std::fabs(out.cmd.vy) < 0.02 && best_cmd.vy == 0.0) out.cmd.vy = 0;
    return out;
  }

  // Distance from the body rectangle (at (x, y, th) in the robot frame) to one point; 0 when the point is inside.
  double point_dist(const Vec2& q, double x, double y, double th) const {
    const double c = std::cos(th), s = std::sin(th);
    const double dx = q.x - x, dy = q.y - y;
    const double lx = c * dx + s * dy, ly = -s * dx + c * dy;
    const double ex = std::max(std::fabs(lx) - p_.hl, 0.0), ey = std::max(std::fabs(ly) - p_.hw, 0.0);
    return std::sqrt(ex * ex + ey * ey);
  }

 private:
  FollowParams p_;
  size_t hint_ = 0;
};

// Turn on the spot toward yaw_err (target - current, wrapped): proportional, never below `min_w` until inside `tol`.
// Returns the angular command; 0 when inside the tolerance.
inline double turn_command(double yaw_err, double wmax, double scale, double prev_wz, double tick_s, double ang_accel, double tol,
                           double min_w = 0.15) {
  if (std::fabs(yaw_err) <= tol) return 0.0;
  const double top = wmax * scale;
  double w = clampd(1.5 * yaw_err, -top, top);
  if (std::fabs(w) < min_w) w = std::copysign(std::min(min_w, top), yaw_err);
  const double dw = ang_accel * tick_s;
  return prev_wz + clampd(w - prev_wz, -dw, dw);
}

}  // namespace go2_nav
