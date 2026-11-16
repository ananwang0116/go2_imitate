#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <random>

#include "go2_nav/config.hpp"
#include "go2_nav/costmap.hpp"
#include "go2_nav/checker.hpp"
#include "go2_nav/follower.hpp"
#include "go2_nav/pathcheck.hpp"
#include "go2_nav/pathmode.hpp"
#include "go2_nav/plantrace.hpp"
#include "go2_nav/planner.hpp"
#include "go2_nav/relay.hpp"
#include "go2_nav/smoother.hpp"

using namespace go2_nav;

namespace {

Geometry geo() {
  return Geometry{0.35, 0.20, 0.10, 0.5, 0.2, 0.10, 0.20, 0.10, 0.30, 0.40};
}
Speeds spd() { return Speeds{0.5, 0.3, 0.2, 0.5, 0.3}; }

// 10 m x 10 m room, 5 cm cells, free inside, a 1-cell wall around
Grid room(double size = 10.0) {
  const int n = static_cast<int>(size / 0.05);
  Grid g(n, n, 0.05, 0.0, 0.0, 0);
  for (int i = 0; i < n; ++i) g.data[g.idx(i, 0)] = g.data[g.idx(i, n - 1)] = g.data[g.idx(0, i)] = g.data[g.idx(n - 1, i)] = 100;
  return g;
}
void rect(Grid& g, double x0, double y0, double x1, double y1, int8_t v = 100) {
  for (int j = static_cast<int>(y0 / g.res); j < static_cast<int>(y1 / g.res); ++j)
    for (int i = static_cast<int>(x0 / g.res); i < static_cast<int>(x1 / g.res); ++i)
      if (g.inside(i, j)) g.data[g.idx(i, j)] = v;
}

}  // namespace

TEST(Edt, MatchesBruteForce) {
  std::mt19937 rng(7);
  const int w = 37, h = 29;
  std::vector<uint8_t> b(w * h, 0);
  for (int k = 0; k < 25; ++k) b[rng() % b.size()] = 1;
  const auto d = distance_m(b, w, h, 0.05);
  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i) {
      double best = 1e18;
      for (int q = 0; q < h; ++q)
        for (int p = 0; p < w; ++p)
          if (b[q * w + p]) best = std::min(best, std::hypot(double(i - p), double(j - q)));
      EXPECT_NEAR(d[j * w + i], best * 0.05, 1e-4) << i << "," << j;
    }
}

TEST(Edt, NoSourcesIsFar) {
  std::vector<uint8_t> b(100, 0);
  EXPECT_GT(distance_m(b, 10, 10, 0.05)[5], 1e5);
}

TEST(Config, LoadsAndRejects) {
  const std::string p = "/tmp/go2_nav_test_geometry.yaml";
  {
    std::ofstream f(p);
    f << "body_half_length_m: 0.35\nbody_half_width_m: 0.2\nloc_sigma_m: 0.1\nbrake_decel_mps2: 0.5\nbrake_reaction_s: 0.2\n"
         "safe_stop_gap_m: 0.1\navoid_gap_m: 0.2\nwaypoint_margin_m: 0.1\nwaypoint_clearance_m: 0.3\narrive_radius_m: 0.4\n"
         "mid360_x_m: 0.3\nmid360_y_m: 0\nmid360_z_m: 0.1\nmid360_roll_deg: 0\nmid360_pitch_deg: -58\nmid360_yaw_deg: 180\n";
  }
  EXPECT_DOUBLE_EQ(load_geometry(p).arrive_radius_m, 0.4);
  {
    std::ofstream f(p, std::ios::app);
    f << "extra: 1\n";
  }
  EXPECT_THROW(load_geometry(p), std::runtime_error);
  // the real geometry.yaml loads too: the key list here and go2_common/geometry.py must agree (mid360_* added 2026-10-30)
  const std::string root = __FILE__;
  EXPECT_DOUBLE_EQ(load_geometry(root.substr(0, root.rfind("/go2_nav/")) + "/go2_bringup/config/geometry.yaml").body_half_length_m, 0.35);
  // the real speeds.yaml loads with the same keys as go2_common/speeds.py (manual_side_mps added 2026-10-26)
  const std::string here = __FILE__;
  const Speeds s = load_speeds(here.substr(0, here.rfind("/go2_nav/")) + "/go2_bringup/config/speeds.yaml");
  EXPECT_DOUBLE_EQ(s.forward_mps, 0.5);
  EXPECT_GE(s.manual_side_mps, s.side_mps);
}

TEST(Planner, OpenRoomIsNearlyStraight) {
  const Grid g = room();
  const CostMap cm = build_costmap(g);
  Planner pl(cm, PlanParams::from_geometry(geo()));
  const auto r = pl.plan({1.5, 1.5}, {8.5, 8.0});
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_NEAR(r.length_m, std::hypot(7.0, 6.5), 0.3);
  for (const Vec2& p : r.path) EXPECT_GE(cm.clearance(p.x, p.y), 0.29);
  EXPECT_NEAR(r.path.front().x, 1.5, 1e-9);
  EXPECT_NEAR(r.path.back().y, 8.0, 1e-9);
}

TEST(Planner, GoesThroughTheGapAndKeepsClearance) {
  Grid g = room();
  rect(g, 4.9, 0.0, 5.1, 4.0);
  rect(g, 4.9, 5.2, 5.1, 10.0);          // a 1.2 m door at y 4.0..5.2
  const CostMap cm = build_costmap(g);
  Planner pl(cm, PlanParams::from_geometry(geo()));
  const auto r = pl.plan({1.5, 1.5}, {8.5, 8.5});
  ASSERT_TRUE(r.ok) << r.reason;
  bool through = false;
  for (const Vec2& p : r.path) {
    EXPECT_GE(cm.clearance(p.x, p.y), 0.29) << p.x << "," << p.y;
    if (std::fabs(p.x - 5.0) < 0.05) through = p.y > 4.0 && p.y < 5.2;
  }
  EXPECT_TRUE(through);
}

TEST(Planner, NarrowDoorBlocksWideDoorPasses) {
  for (auto [gap, expect_ok] : {std::pair{0.5, false}, std::pair{0.8, true}}) {
    Grid g = room();
    rect(g, 4.9, 0.0, 5.1, 4.5);
    rect(g, 4.9, 4.5 + gap, 5.1, 10.0);
    const CostMap cm = build_costmap(g);
    Planner pl(cm, PlanParams::from_geometry(geo()));
    const auto r = pl.plan({1.5, 1.5}, {8.5, 8.5});
    EXPECT_EQ(r.ok, expect_ok) << "gap " << gap << " " << r.reason;
    if (!expect_ok) {
      EXPECT_EQ(r.reason, "no_path");
    }
  }
}

TEST(Planner, StartInsideTheSoftZoneStillLeaves) {
  Grid g = room();
  const CostMap cm = build_costmap(g);
  Planner pl(cm, PlanParams::from_geometry(geo()));
  const auto r = pl.plan({0.5, 5.0}, {5.0, 5.0});       // 0.43 m from the wall: fine; now closer:
  ASSERT_TRUE(r.ok);
  const auto r2 = pl.plan({0.27, 5.0}, {5.0, 5.0});     // centre 0.22 m from the wall face: inside the hard band
  ASSERT_TRUE(r2.ok) << r2.reason;
  EXPECT_GT(r2.path.back().x, 4.9);
}

TEST(Planner, GoalInsideWallSnapsOrFails) {
  Grid g = room();
  rect(g, 6.0, 4.0, 6.4, 6.0);
  const CostMap cm = build_costmap(g);
  Planner pl(cm, PlanParams::from_geometry(geo()));
  const auto r = pl.plan({2.0, 5.0}, {6.2, 5.0});
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_GE(cm.clearance(r.path.back().x, r.path.back().y), 0.3);
  EXPECT_LE(std::hypot(r.path.back().x - 6.2, r.path.back().y - 5.0), 0.7);
  Grid g2 = room();
  rect(g2, 5.0, 3.0, 8.0, 7.0);                         // a big block: nothing passable within 0.6 m of the goal
  const CostMap cm2 = build_costmap(g2);
  Planner pl2(cm2, PlanParams::from_geometry(geo()));
  EXPECT_EQ(pl2.plan({2.0, 5.0}, {6.5, 5.0}).reason, "goal_blocked");
}

TEST(Planner, UnknownIsPassable) {
  Grid g = room();
  rect(g, 4.0, 0.5, 6.0, 9.5, -1);                      // unknown band across the room
  const CostMap cm = build_costmap(g);
  Planner pl(cm, PlanParams::from_geometry(geo()));
  EXPECT_TRUE(pl.plan({1.5, 5.0}, {8.5, 5.0}).ok);
}

// A saved-map wall nobody has seen this run (60) may be gone (something stood there while mapping): passable, but the path
// takes a door whenever there is one, keeps away from it like from a real wall, and the laser decides (user decision 2026-10-26).
TEST(Planner, UnseenPriorWallIsADetourNotABarrier) {
  // fully closed: a real (100) wall = no path; an unseen prior wall (60) = the dog may go and look
  Grid closed = room();
  rect(closed, 4.9, 0.0, 5.1, 10.0, 100);
  const CostMap cm_c = build_costmap(closed);
  EXPECT_EQ(Planner(cm_c, PlanParams::from_geometry(geo())).plan({2.5, 5.0}, {7.5, 5.0}).reason, "no_path");
  Grid closed60 = room();
  rect(closed60, 4.9, 0.0, 5.1, 10.0, 60);
  const CostMap cm_c60 = build_costmap(closed60);
  EXPECT_TRUE(Planner(cm_c60, PlanParams::from_geometry(geo())).plan({2.5, 5.0}, {7.5, 5.0}).ok);

  // with a door 3.5 m to the side the unseen wall is NOT crossed: the door wins (2026-10-26 it went straight through)
  Grid door = room();
  rect(door, 4.9, 0.0, 5.1, 8.0, 60);
  rect(door, 4.9, 9.0, 5.1, 10.0, 60);                  // 1 m door at y 8..9
  const CostMap cm_d = build_costmap(door);
  const auto r = Planner(cm_d, PlanParams::from_geometry(geo())).plan({2.5, 4.5}, {7.5, 4.5});
  ASSERT_TRUE(r.ok) << r.reason;
  for (const Vec2& q : r.path)
    if (std::fabs(q.x - 5.0) < 0.1) {
      EXPECT_GT(q.y, 7.9) << "crossed the wall at y " << q.y;
    }

  // seen floor (evidence 0) where the wall was: it is gone, straight through
  Grid gone = door;
  rect(gone, 4.9, 4.0, 5.1, 5.0, 0);
  const CostMap cm_g = build_costmap(gone);
  const auto r2 = Planner(cm_g, PlanParams::from_geometry(geo())).plan({2.5, 4.5}, {7.5, 4.5});
  ASSERT_TRUE(r2.ok);
  EXPECT_LT(r2.length_m, 6.0);
}

TEST(Planner, KeepsAwayFromUnseenPriorWalls) {
  Grid g = room();
  rect(g, 0.5, 3.0, 9.5, 3.1, 60);                      // a 1.5 m corridor between two unseen prior walls
  rect(g, 0.5, 4.6, 9.5, 4.7, 60);
  const CostMap cm = build_costmap(g);
  const auto r = Planner(cm, PlanParams::from_geometry(geo())).plan({1.0, 3.85}, {9.0, 3.85});
  ASSERT_TRUE(r.ok) << r.reason;
  for (const Vec2& q : r.path) EXPECT_GE(cm.prior_clearance(q.x, q.y), 0.29) << q.x << "," << q.y;
}

TEST(Checker, UnseenPriorWallDoesNotFailTheCheckButARealOneDoes) {
  for (int v : {60, 100}) {
    Grid g = room();
    rect(g, 4.9, 0.0, 5.1, 10.0, static_cast<int8_t>(v));
    const CostMap cm = build_costmap(g);
    CheckIn in;
    in.pos = {{7.5, 5.0}};
    in.radius = {0.4};
    in.paths = {{}};
    in.use_start = true;
    in.start = {2.5, 5.0};
    const CheckOut out = check_waypoints(cm, PlanParams::from_geometry(geo()), geo(), in);
    EXPECT_EQ(out.ok[0], v == 60) << v << ": " << out.reason[0];
  }
}

TEST(Planner, FreshScanPointsBlock) {
  const Grid g = room();
  std::vector<Vec2> pts;
  for (double y = 0.5; y < 9.5; y += 0.05) pts.push_back({5.0, y});     // a wall of points seen by the laser
  const CostMap cm = build_costmap(g, pts);
  Planner pl(cm, PlanParams::from_geometry(geo()));
  EXPECT_EQ(pl.plan({1.5, 5.0}, {8.5, 5.0}).reason, "no_path");
}

TEST(Planner, PerformanceOnABigMap) {
  Grid g = room(30.0);                                  // 600 x 600 cells
  rect(g, 14.9, 0.0, 15.1, 14.0);
  rect(g, 14.9, 16.0, 15.1, 30.0);
  const CostMap cm = build_costmap(g);
  Planner pl(cm, PlanParams::from_geometry(geo()));
  const auto t0 = std::chrono::steady_clock::now();
  const auto r = pl.plan({2.0, 2.0}, {28.0, 28.0});
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  ASSERT_TRUE(r.ok);
  EXPECT_LT(ms, 300.0) << "plan took " << ms << " ms, expanded " << r.expanded;
}

namespace {
std::vector<Vec2> blocked_points(const Grid& g, const Pose2& p, double radius) {
  std::vector<Vec2> out;
  int ci, cj;
  g.cell(p.x, p.y, ci, cj);
  const int r = static_cast<int>(radius / g.res);
  for (int j = cj - r; j <= cj + r; ++j)
    for (int i = ci - r; i <= ci + r; ++i)
      if (g.inside(i, j) && g.blocked(i, j)) {
        const Vec2 c = g.center(i, j);
        if (std::hypot(c.x - p.x, c.y - p.y) <= radius) out.push_back(c);
      }
  return out;
}

// closed loop: plan, follow, integrate the body at 20 Hz, scan = blocked cells within 4 m. Returns seconds to arrive, -1 = failed.
double drive(const Grid& world, Pose2 pose, Vec2 goal, double& min_clear, bool& was_blocked) {
  const CostMap cm = build_costmap(world);
  Planner planner(cm, PlanParams::from_geometry(geo()));
  Follower fol(FollowParams::from(geo(), spd()));
  auto plan = planner.plan({pose.x, pose.y}, goal);
  if (!plan.ok) return -1;
  fol.reset();
  Cmd prev;
  min_clear = 1e9;
  was_blocked = false;
  const double dt = 0.05;
  for (int k = 0; k < 20 * 120; ++k) {
    if (std::hypot(goal.x - pose.x, goal.y - pose.y) <= 0.4) return k * dt;
    if (k % 20 == 0 && k > 0) {
      auto p2 = planner.plan({pose.x, pose.y}, goal);
      if (p2.ok) {
        plan = p2;
        fol.reset();
      }
    }
    const auto pts = blocked_points(world, pose, 4.0);
    const auto o = fol.step(pose, plan.path, pts, &cm, 1.0, prev, dt);
    if (o.blocked) {
      was_blocked = true;
      return -1;
    }
    prev = o.cmd;
    pose.x += (o.cmd.vx * std::cos(pose.yaw) - o.cmd.vy * std::sin(pose.yaw)) * dt;
    pose.y += (o.cmd.vx * std::sin(pose.yaw) + o.cmd.vy * std::cos(pose.yaw)) * dt;
    pose.yaw += o.cmd.wz * dt;
    min_clear = std::min(min_clear, cm.clearance(pose.x, pose.y));
  }
  return -1;
}
}  // namespace

namespace {
struct RunResult {
  bool arrived = false, blocked = false;
  double seconds = 0, min_body_gap = 1e9;
};
// plan on `planned`, follow with the scan of `world`; replan_s <= 0 = never replan; the replan sees the fresh scan points
RunResult follow_run(const Grid& planned, const Grid& world, Pose2 pose, Vec2 goal, double replan_s, double max_s = 90.0) {
  RunResult r;
  const CostMap cm_w = build_costmap(world);
  CostMap cm_p = build_costmap(planned);
  Planner pl(cm_p, PlanParams::from_geometry(geo()));
  auto plan = pl.plan({pose.x, pose.y}, goal);
  if (!plan.ok) return r;
  Follower fol(FollowParams::from(geo(), spd()));
  Cmd prev;
  const double dt = 0.05;
  for (int k = 0; k * dt < max_s; ++k) {
    r.seconds = k * dt;
    if (std::hypot(goal.x - pose.x, goal.y - pose.y) <= 0.4) {
      r.arrived = true;
      return r;
    }
    const auto pts = blocked_points(world, pose, 4.0);
    if (replan_s > 0 && k % static_cast<int>(replan_s / dt) == 0 && k > 0) {
      cm_p = build_costmap(planned, pts);
      Planner again(cm_p, PlanParams::from_geometry(geo()));
      const auto p2 = again.plan({pose.x, pose.y}, goal);
      if (p2.ok) {
        plan = p2;
        fol.reset();
      }
    }
    const auto o = fol.step(pose, plan.path, pts, &cm_w, 1.0, prev, dt);
    if (o.blocked) {
      r.blocked = true;
      return r;
    }
    prev = o.cmd;
    pose.x += (o.cmd.vx * std::cos(pose.yaw) - o.cmd.vy * std::sin(pose.yaw)) * dt;
    pose.y += (o.cmd.vx * std::sin(pose.yaw) + o.cmd.vy * std::cos(pose.yaw)) * dt;
    pose.yaw += o.cmd.wz * dt;
    // gap from the body rectangle to the world's blocked cells
    double gap = 1e9;
    for (const Vec2& q : blocked_points(world, pose, 1.0)) {
      const double dx = q.x - pose.x, dy = q.y - pose.y;
      const double lx = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy, ly = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
      gap = std::min(gap, std::hypot(std::max(std::fabs(lx) - 0.35, 0.0), std::max(std::fabs(ly) - 0.20, 0.0)));
    }
    r.min_body_gap = std::min(r.min_body_gap, gap);
  }
  return r;
}
}  // namespace

TEST(Follower, UnknownBoxWithoutReplanStopsBeforeItAndNeverTouches) {
  Grid world = room();
  const Grid planned = world;
  rect(world, 4.6, 3.0, 5.4, 7.0);                      // a box that the plan does not know (the laser sees it)
  const RunResult r = follow_run(planned, world, {1.5, 5.0, 0.0}, {8.5, 5.0}, 0.0);
  EXPECT_FALSE(r.arrived);
  EXPECT_GE(r.min_body_gap, 0.05) << "the body touched the box";
}

TEST(Follower, UnknownBoxWithReplanGoesAround) {
  Grid world = room();
  const Grid planned = world;
  rect(world, 4.6, 3.0, 5.4, 7.0);
  const RunResult r = follow_run(planned, world, {1.5, 5.0, 0.0}, {8.5, 5.0}, 1.0);
  EXPECT_TRUE(r.arrived) << "blocked=" << r.blocked << " after " << r.seconds << " s";
  EXPECT_GE(r.min_body_gap, 0.05);
}

TEST(Follower, BlockedWhenBoxedIn) {
  Grid world = room();
  const CostMap cm = build_costmap(world);
  Follower fol(FollowParams::from(geo(), spd()));
  std::vector<Vec2> pts;
  for (double a = 0; a < 6.28; a += 0.05) pts.push_back({5.0 + 0.5 * std::cos(a), 5.0 + 0.5 * std::sin(a)});   // ring at 0.5 m
  // the body half length is 0.35, margin 0.10: a ring at 0.5 m is outside -> still free to move; shrink it
  pts.clear();
  for (double a = 0; a < 6.28; a += 0.05) pts.push_back({5.0 + 0.40 * std::cos(a), 5.0 + 0.40 * std::sin(a)});
  std::vector<Vec2> path{{5.0, 5.0}, {6.0, 5.0}, {7.0, 5.0}};
  const auto o = fol.step({5.0, 5.0, 0.0}, path, pts, &cm, 1.0, Cmd{}, 0.05);
  // ring closer than the body corners: points are inside the body already (init clearance 0): moving away is allowed only
  // if it does not get worse; every direction gets worse -> blocked
  EXPECT_TRUE(o.blocked);
  EXPECT_EQ(o.cmd.vx, 0.0);
}

TEST(Follower, TurnsInPlaceWhenFacingAway) {
  Grid world = room();
  const CostMap cm = build_costmap(world);
  Follower fol(FollowParams::from(geo(), spd()));
  std::vector<Vec2> path{{5.0, 5.0}, {4.0, 5.0}, {3.0, 5.0}, {2.0, 5.0}};
  const auto o = fol.step({5.0, 5.0, 0.0}, path, {}, &cm, 1.0, Cmd{}, 0.05);   // goal is behind
  EXPECT_FALSE(o.blocked);
  EXPECT_NEAR(o.cmd.vx, 0.0, 1e-9);
  EXPECT_NE(o.cmd.wz, 0.0);
}

TEST(Follower, SpeedsRespectLimitsAndMinMove) {
  Grid world = room();
  const CostMap cm = build_costmap(world);
  Follower fol(FollowParams::from(geo(), spd()));
  std::vector<Vec2> path;
  for (double x = 2.0; x <= 8.0; x += 0.1) path.push_back({x, 5.0});
  Cmd prev;
  for (int k = 0; k < 200; ++k) {
    const auto o = fol.step({2.0, 5.0, 0.0}, path, {}, &cm, 1.0, prev, 0.05);
    prev = o.cmd;
    EXPECT_LE(o.cmd.vx, 0.5 + 1e-9);
    EXPECT_LE(std::fabs(o.cmd.vy), 0.15 + 1e-9);
    EXPECT_LE(std::fabs(o.cmd.wz), 0.5 + 1e-9);
  }
  EXPECT_NEAR(prev.vx, 0.5, 0.05);                       // it reaches the speed limit on a free straight path
  Cmd q;
  for (int k = 0; k < 200; ++k) q = fol.step({2.0, 5.0, 0.0}, path, {}, &cm, 0.5, q, 0.05).cmd;     // speed_pct 50
  EXPECT_NEAR(q.vx, 0.25, 0.03);
}

TEST(ClosedLoop, CrossesTheRoomThroughADoor) {
  Grid world = room();
  rect(world, 4.9, 0.0, 5.1, 4.0);
  rect(world, 4.9, 5.2, 5.1, 10.0);
  double clear = 0;
  bool blocked = false;
  const double t = drive(world, {1.5, 1.5, 1.0}, {8.5, 8.5}, clear, blocked);
  EXPECT_GT(t, 0) << "did not arrive (blocked=" << blocked << ")";
  EXPECT_LT(t, 60.0);
  EXPECT_GE(clear, 0.25) << "the centre came too close to a wall";
}

// ---------------- path goals, checker, final turn ----------------
namespace {
std::vector<Vec2> line(Vec2 a, Vec2 b, double step = 0.1) {
  std::vector<Vec2> out;
  const double L = std::hypot(b.x - a.x, b.y - a.y);
  for (double d = 0; d < L; d += step) out.push_back({a.x + (b.x - a.x) * d / L, a.y + (b.y - a.y) * d / L});
  out.push_back(b);
  return out;
}
}  // namespace

TEST(PathMode, OnThePathJustFollowsIt) {
  const Grid g = room();
  const CostMap cm = build_costmap(g);
  const auto taught = line({1.5, 5.0}, {8.0, 5.0});
  size_t hint = 0;
  const auto r = plan_on_taught(cm, PlanParams::from_geometry(geo()), TaughtParams(), taught, {3.0, 5.2}, hint, {}, 0.2, 0.4);
  ASSERT_TRUE(r.ok);
  EXPECT_FALSE(r.joined);
  EXPECT_FALSE(r.detoured);
  EXPECT_EQ(r.path.size(), taught.size() - r.k0 + 1);           // the dog + the rest of the taught path, unchanged
  EXPECT_NEAR(r.path.back().x, 8.0, 1e-9);
  EXPECT_GT(r.k0, 10u);
}

TEST(PathMode, FarFromThePathJoinsWithAStar) {
  const Grid g = room();
  const CostMap cm = build_costmap(g);
  const auto taught = line({1.5, 5.0}, {8.0, 5.0});
  size_t hint = 0;
  const auto r = plan_on_taught(cm, PlanParams::from_geometry(geo()), TaughtParams(), taught, {3.0, 7.5}, hint, {}, 0.2, 0.4);
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_TRUE(r.joined);
  EXPECT_NEAR(r.path.back().x, 8.0, 1e-9);
  EXPECT_NEAR(r.path.front().y, 7.5, 1e-9);
}

TEST(PathMode, BlockedStretchIsDetouredAndRejoined) {
  Grid g = room();
  const CostMap cm0 = build_costmap(g);
  const auto taught = line({1.5, 5.0}, {8.0, 5.0});
  std::vector<Vec2> fresh;                                      // a box the laser sees on the path at x 4.4..4.6
  for (double y = 4.0; y <= 6.0; y += 0.05) {
    fresh.push_back({4.5, y});
    fresh.push_back({4.4, y});
  }
  const CostMap cm = build_costmap(g, fresh);
  size_t hint = 0;
  const auto r = plan_on_taught(cm, PlanParams::from_geometry(geo()), TaughtParams(), taught, {3.2, 5.0}, hint, fresh, 0.2, 0.4);
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_TRUE(r.detoured);
  EXPECT_NEAR(r.path.back().x, 8.0, 1e-9);
  for (const Vec2& p : r.path) EXPECT_GE(cm.clearance(p.x, p.y), 0.19) << p.x << "," << p.y;
  double max_dev = 0;
  for (const Vec2& p : r.path) max_dev = std::max(max_dev, std::fabs(p.y - 5.0));
  EXPECT_GT(max_dev, 0.5);                                      // it really went around
  (void)cm0;
}

TEST(PathMode, IndexNeverJumpsBackOrAcrossALoop) {
  std::vector<Vec2> p = line({0, 0}, {5, 0});
  const auto back = line({5, 0.2}, {0, 0.2});
  p.insert(p.end(), back.begin(), back.end());                  // out and back, 0.2 m apart
  const size_t k = nearest_index(p, {2.0, 0.1}, 0, 80);
  EXPECT_LE(k, 40u);                                            // on the way out, not the way back
  EXPECT_GT(nearest_index(p, {2.0, 0.1}, 60, 80), 60u);
}

TEST(Checker, ReachableWaypointsAndWhyNot) {
  Grid g = room();
  rect(g, 4.9, 0.0, 5.1, 9.0);                                  // a wall with a gap only at y 9..10 (1 m)
  rect(g, 8.0, 8.0, 9.0, 9.0, -1);                              // an unknown patch
  const CostMap cm = build_costmap(g);
  CheckIn in;
  in.pos = {{2.0, 2.0}, {2.5, 8.5}, {8.5, 2.0}, {50.0, 2.0}, {8.5, 6.5}};
  in.radius = {0.4, 0.4, 0.4, 0.4, 0.4};
  in.use_start = true;
  in.start = {1.5, 1.5};
  const CheckOut o = check_waypoints(cm, PlanParams::from_geometry(geo()), geo(), in);
  ASSERT_EQ(o.ok.size(), 5u);
  EXPECT_TRUE(o.ok[0]);
  EXPECT_TRUE(o.ok[1]) << o.reason[1];
  EXPECT_TRUE(o.ok[2]) << o.reason[2];                          // around through the gap
  EXPECT_GT(o.leg_m[2], 10.0);                                  // and it is a long way from waypoint 1: up to the gap and back down
  ASSERT_EQ(o.leg_path.size(), 5u);                             // display geometry of each leg
  ASSERT_GT(o.leg_path[2].size(), 2u);
  EXPECT_LT(hyp(o.leg_path[2].back().x - 8.5, o.leg_path[2].back().y - 2.0), 0.6);
  EXPECT_TRUE(o.leg_path[3].empty());                           // no leg for an unusable waypoint
  EXPECT_FALSE(o.ok[3]);
  EXPECT_EQ(o.reason[3], "outside the map");
  EXPECT_TRUE(o.ok[4]) << o.reason[4];                          // (8.5, 6.5) is clear of the unknown patch
}

TEST(Checker, WallUnknownAndPathProblems) {
  Grid g = room();
  rect(g, 4.9, 0.0, 5.1, 10.0);                                 // a closed wall: the right half is not reachable
  rect(g, 2.0, 2.0, 3.0, 3.0, -1);                              // a big unknown patch
  rect(g, 6.0, 6.0, 7.0, 7.0);                                  // a solid block
  const CostMap cm = build_costmap(g);
  CheckIn in;
  in.pos = {{2.5, 2.5}, {6.5, 6.5}, {8.0, 2.0}, {1.5, 8.0}};
  in.radius = {0.4, 0.4, 0.4, 0.05};
  in.paths = {{}, {}, {}, {}};
  in.use_start = true;
  in.start = {1.5, 1.5};
  const CheckOut o = check_waypoints(cm, PlanParams::from_geometry(geo()), geo(), in);
  EXPECT_FALSE(o.ok[0]);
  EXPECT_NE(o.reason[0].find("unknown"), std::string::npos) << o.reason[0];
  EXPECT_FALSE(o.ok[1]);
  EXPECT_NE(o.reason[1].find("no passable position"), std::string::npos) << o.reason[1];
  EXPECT_FALSE(o.ok[2]);
  EXPECT_NE(o.reason[2].find("not connected"), std::string::npos) << o.reason[2];
  EXPECT_FALSE(o.ok[3]);
  EXPECT_NE(o.reason[3].find("loc_sigma"), std::string::npos) << o.reason[3];
  // a taught path through the block
  CheckIn p;
  p.pos = {{8.0, 2.0}};
  p.radius = {0.4};
  p.paths = {line({6.0, 6.5}, {7.0, 6.5})};
  Grid g2 = room();
  rect(g2, 6.4, 6.0, 6.6, 7.0);
  const CheckOut o2 = check_waypoints(build_costmap(g2), PlanParams::from_geometry(geo()), geo(), p);
  EXPECT_FALSE(o2.ok[0]);
  EXPECT_NE(o2.reason[0].find("taught path blocked"), std::string::npos) << o2.reason[0];
}

TEST(FinalTurn, TurnsTowardTheTargetAndStopsInsideTheTolerance) {
  double w = 0;
  EXPECT_EQ(turn_command(0.05, 0.5, 1.0, 0.3, 0.05, 2.0, 0.12), 0.0);
  for (int k = 0; k < 30; ++k) w = turn_command(1.0, 0.5, 1.0, w, 0.05, 2.0, 0.12);
  EXPECT_NEAR(w, 0.5, 1e-9);
  w = 0;
  for (int k = 0; k < 30; ++k) w = turn_command(-2.0, 0.5, 0.5, w, 0.05, 2.0, 0.12);
  EXPECT_NEAR(w, -0.25, 1e-9);                                  // speed_pct 50
  EXPECT_NEAR(turn_command(0.13, 0.5, 1.0, 0.0, 0.05, 2.0, 0.12), 0.1, 1e-9);   // small error: the rate limit, first step
}

TEST(Planner, PerWaypointRadiusLetsTheGoalEndAnywhereInTheArea) {
  Grid g = room();
  rect(g, 6.0, 4.0, 6.4, 6.0);                                  // goal centre inside the block, area radius 1.0 reaches free cells
  const CostMap cm = build_costmap(g);
  Planner pl(cm, PlanParams::from_geometry(geo()));
  EXPECT_EQ(pl.plan({2.0, 5.0}, {6.2, 5.0}, 0.3).reason, "goal_blocked");
  EXPECT_TRUE(pl.plan({2.0, 5.0}, {6.2, 5.0}, 1.0).ok);
}

// ---------------- docs/05 section 9: two-point footprint, path look-ahead, relay, smoother ----------------
namespace {
// worst footprint clearance of a path (heading = segment direction), skipping the start zone and the goal area
double worst_fp(const CostMap& cm, const PlanParams& pp, const std::vector<Vec2>& path, double skip_head, double skip_tail) {
  double total = path_length(path), s = 0, worst = 1e9;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const double len = hyp(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
    if (len > 1e-9 && s >= skip_head && s <= total - skip_tail)
      worst = std::min(worst, cm.footprint_clearance(path[i].x, path[i].y, std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x), pp.fp_offset_m));
    s += len;
  }
  return worst;
}
// biggest change of direction between two neighbouring 10 cm steps [rad]
double max_turn(const std::vector<Vec2>& p) {
  double worst = 0;
  for (size_t i = 2; i < p.size(); ++i) {
    const double a = std::atan2(p[i - 1].y - p[i - 2].y, p[i - 1].x - p[i - 2].x), b = std::atan2(p[i].y - p[i - 1].y, p[i].x - p[i - 1].x);
    worst = std::max(worst, std::fabs(wrap(b - a)));
  }
  return worst;
}
}  // namespace

TEST(Footprint, DerivedFromTheGeometryWithoutNewKeys) {
  const PlanParams pp = PlanParams::from_geometry(geo());
  EXPECT_NEAR(pp.hard_m, 0.30, 1e-9);
  EXPECT_NEAR(pp.fp_offset_m, 0.15, 1e-9);                       // half length 0.35 - half width 0.20
  EXPECT_NEAR(pp.hard_m + pp.fp_offset_m, 0.35 + 0.10, 1e-9);    // the discs reach half length + safe_stop_gap in front
  const FollowParams fp = FollowParams::from(geo(), spd());
  EXPECT_NEAR(fp.fp_offset, 0.15, 1e-9);
}

TEST(Footprint, ReadsBothCentresAndTheDirectionAwayFromAWall) {
  Grid g = room();
  const CostMap cm = build_costmap(g);
  // facing the east wall at x=10: the front centre is 0.15 closer than the body centre
  const double d_c = cm.clearance(9.0, 5.0);
  EXPECT_NEAR(cm.footprint_clearance(9.0, 5.0, 0.0, 0.15), d_c - 0.15, 0.06);
  EXPECT_NEAR(cm.footprint_clearance(9.0, 5.0, M_PI / 2, 0.15), d_c, 0.06);    // sideways to the wall: both centres as far as the body
  EXPECT_NEAR(cm.footprint_clearance(9.0, 5.0, 0.0, 0.0), d_c, 1e-9);          // offset 0 = the old single disc
  EXPECT_TRUE(std::isnan(cm.footprint_clearance(-1.0, 5.0, 0.0, 0.15)));        // outside the map
  const Vec2 up = cm.clearance_gradient(9.0, 5.0);                              // pointing away from the east wall: toward -x
  EXPECT_LT(up.x, -0.9);
}

TEST(Footprint, PlannedPathKeepsBothCentresClear) {
  Grid g = room();
  rect(g, 4.9, 0.0, 5.1, 4.0);
  rect(g, 4.9, 5.2, 5.1, 10.0);
  rect(g, 6.5, 6.0, 7.5, 7.0);
  const CostMap cm = build_costmap(g);
  const PlanParams pp = PlanParams::from_geometry(geo());
  Planner pl(cm, pp);
  const auto r = pl.plan({1.5, 1.5}, {8.5, 8.5}, 0.4);
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_GE(worst_fp(cm, pp, r.path, 0.6, 0.4), 0.24) << "the front or rear centre came closer than the hard radius";
}

TEST(Footprint, ChangesTheDecisionAtADeadEndWall) {
  // a wall straight across the corridor end: the goal sits 0.40 m in front of it. The old disc (centre >= 0.30) accepted
  // standing there heading INTO the wall; the two discs need the front centre 0.30 off the wall = centre 0.45.
  Grid g = room();
  const CostMap cm = build_costmap(g);
  PlanParams one = PlanParams::from_geometry(geo());
  one.fp_offset_m = 0.0;
  const PlanParams two = PlanParams::from_geometry(geo());
  Planner p1(cm, one), p2(cm, two);
  const auto a = p1.plan({5.0, 5.0}, {9.6, 5.0}, 0.05);        // goal area is only the point itself
  const auto b = p2.plan({5.0, 5.0}, {9.6, 5.0}, 0.05);
  ASSERT_TRUE(a.ok && b.ok) << a.reason << " " << b.reason;
  // approaching along +x, the last stretch (outside the 5 cm goal area) of the two-disc path stays 0.45 m from the wall
  double min_x_gap_two = 1e9;
  for (const Vec2& q : b.path) if (hyp(q.x - 9.6, q.y - 5.0) > 0.1) min_x_gap_two = std::min(min_x_gap_two, 9.95 - q.x);
  EXPECT_GE(min_x_gap_two, 0.30);
}

TEST(PathCheck, FindsTheFirstHitAsADistanceAlongThePath) {
  Grid g = room();
  rect(g, 6.0, 4.0, 6.2, 6.0);                                   // a wall across the path at x = 6
  const CostMap cm = build_costmap(g);
  const PlanParams pp = PlanParams::from_geometry(geo());
  const std::vector<Vec2> path = line({2.0, 5.0}, {9.0, 5.0});
  PathCheckParams pc;
  pc.horizon_m = 6.0;
  const PathHit h = first_hit(&cm, pp, pc, path, {2.0, 5.0}, {});
  ASSERT_TRUE(h.hit);
  // the front centre meets 0.30 * 0.9 clearance when it is 0.27 + 0.15 = 0.42 m before the wall face (x = 6.0): body centre at 5.58
  EXPECT_NEAR(h.dist_m, 3.58, 0.25);
  pc.horizon_m = 3.0;
  EXPECT_FALSE(first_hit(&cm, pp, pc, path, {2.0, 5.0}, {}).hit);   // beyond the horizon
  // a fresh scan point right on the path: found at once, whatever the map says
  const std::vector<Vec2> fresh{{3.5, 5.0}};
  const PathHit f = first_hit(&cm, pp, pc, path, {2.0, 5.0}, fresh);
  ASSERT_TRUE(f.hit);
  EXPECT_NEAR(f.dist_m, 1.5 - 0.15 - 0.25, 0.2);
  // a point well beside the path does not count
  EXPECT_FALSE(first_hit(&cm, pp, pc, path, {2.0, 5.0}, {{3.5, 5.6}}).hit);
}

TEST(PathCheck, NothingToCheckInTheGoalAreaOrAtTheStart) {
  Grid g = room();
  const CostMap cm = build_costmap(g);
  const PlanParams pp = PlanParams::from_geometry(geo());
  const std::vector<Vec2> path = line({8.0, 5.0}, {9.65, 5.0});      // ends 0.3 m from the east wall: the goal area
  PathCheckParams pc;
  pc.tail_free_m = 0.6;
  EXPECT_FALSE(first_hit(&cm, pp, pc, path, {8.0, 5.0}, {}).hit);
  pc.tail_free_m = 0.0;
  EXPECT_TRUE(first_hit(&cm, pp, pc, path, {8.0, 5.0}, {}).hit);
  // the dog hugging a wall at the start is not a hit (its own clearance rules in the first near_m)
  EXPECT_FALSE(first_hit(&cm, pp, pc, line({0.3, 5.0}, {0.3, 5.5}), {0.3, 5.0}, {}).hit);
}

TEST(Relay, KeepsTheNearPartAndOnlyReplansBeyondIt) {
  Grid g = room();
  const CostMap cm0 = build_costmap(g);
  const PlanParams pp = PlanParams::from_geometry(geo());
  RelayParams rp;
  const RelayPlan first = plan_relay(cm0, pp, rp, {}, {1.5, 5.0}, {8.5, 5.0}, 0.4);
  ASSERT_TRUE(first.ok);
  EXPECT_FALSE(first.relayed);                                    // nothing to relay from
  // an obstacle appears 3 m ahead of the dog, off to one side of the old straight path
  Grid g2 = g;
  rect(g2, 5.0, 4.8, 5.4, 6.0);
  const CostMap cm1 = build_costmap(g2);
  const Vec2 dog{2.5, 5.0};
  const RelayPlan again = plan_relay(cm1, pp, rp, first.path, dog, {8.5, 5.0}, 0.4);
  ASSERT_TRUE(again.ok) << again.reason;
  EXPECT_TRUE(again.relayed);
  EXPECT_NEAR(again.kept_m, 1.0, 0.15);
  // the first metre is the old path, point for point
  for (size_t i = 1; i < again.path.size() && hyp(again.path[i].x - dog.x, again.path[i].y - dog.y) < 0.9; ++i)
    EXPECT_NEAR(again.path[i].y, 5.0, 1e-6) << "the near part moved at " << again.path[i].x;
  // and it still gets around the new box
  EXPECT_GE(worst_fp(cm1, pp, again.path, 0.6, 0.4), 0.24);
  EXPECT_NEAR(again.path.back().x, 8.5, 1e-6);
}

TEST(Relay, FallsBackToAPlainPlanWhenTheKeptStretchIsBlockedOrTheDogIsOff) {
  Grid g = room();
  const CostMap cm0 = build_costmap(g);
  const PlanParams pp = PlanParams::from_geometry(geo());
  RelayParams rp;
  const RelayPlan first = plan_relay(cm0, pp, rp, {}, {1.5, 5.0}, {8.5, 5.0}, 0.4);
  ASSERT_TRUE(first.ok);
  Grid g2 = g;
  rect(g2, 3.3, 4.7, 3.7, 5.3);                                   // right on the metre that would be kept
  const RelayPlan blocked = plan_relay(build_costmap(g2), pp, rp, first.path, {2.5, 5.0}, {8.5, 5.0}, 0.4);
  ASSERT_TRUE(blocked.ok) << blocked.reason;
  EXPECT_FALSE(blocked.relayed);
  const RelayPlan off = plan_relay(cm0, pp, rp, first.path, {2.5, 6.5}, {8.5, 5.0}, 0.4);   // 1.5 m off the old path
  ASSERT_TRUE(off.ok);
  EXPECT_FALSE(off.relayed);
  const RelayPlan nearly_done = plan_relay(cm0, pp, rp, first.path, {8.2, 5.0}, {8.5, 5.0}, 0.4);
  EXPECT_FALSE(nearly_done.relayed);
}

TEST(Smoother, RoundsTheCornerAndNeverMakesItWorse) {
  Grid g = room();
  rect(g, 3.0, 0.0, 3.4, 6.0);
  rect(g, 6.0, 4.0, 6.4, 10.0);                                    // an S-shaped way: left of the first wall, right of the second
  const CostMap cm = build_costmap(g);
  const PlanParams pp = PlanParams::from_geometry(geo());
  Planner pl(cm, pp);
  const auto r = pl.plan({1.5, 1.5}, {8.5, 8.5}, 0.4);
  ASSERT_TRUE(r.ok) << r.reason;
  SmoothParams sp;
  sp.skip_tail_m = 0.4;
  const SmoothResult sm = smooth_path(cm, pp, sp, r.path);
  ASSERT_TRUE(sm.ok) << sm.why;
  EXPECT_NEAR(sm.path.front().x, r.path.front().x, 1e-6);          // starts at the dog ...
  EXPECT_NEAR(sm.path.back().y, r.path.back().y, 1e-6);            // ... ends at the goal
  EXPECT_NEAR(sm.path.back().x, r.path.back().x, 1e-6);
  EXPECT_LT(max_turn(sm.path), max_turn(smooth_detail::resample_arc(r.path, 0.1)) + 1e-9);
  EXPECT_GE(sm.out_min_m, std::min(pp.hard_m - 0.02, sm.in_min_m) - 1e-6);
  EXPECT_LT(path_length(sm.path), 1.1 * r.length_m + 0.3);
}

TEST(Smoother, TurnsAreSofterThanThePolyline) {
  Grid g = room();
  rect(g, 4.0, 0.0, 4.4, 7.0);                                     // forces a sharp doubling back around the end of a wall
  const CostMap cm = build_costmap(g);
  const PlanParams pp = PlanParams::from_geometry(geo());
  Planner pl(cm, pp);
  const auto r = pl.plan({2.0, 2.0}, {6.5, 2.0}, 0.4);
  ASSERT_TRUE(r.ok) << r.reason;
  const auto raw = smooth_detail::resample_arc(r.path, 0.1);
  const SmoothResult sm = smooth_path(cm, pp, SmoothParams(), r.path);
  ASSERT_TRUE(sm.ok) << sm.why;
  EXPECT_LT(max_turn(sm.path), 0.7 * max_turn(raw) + 1e-9) << "raw " << max_turn(raw) << " smooth " << max_turn(sm.path);
  // never closer to the wall end than the polyline was (the push-off term)
  EXPECT_GE(sm.out_min_m, std::min(pp.hard_m - 0.02, sm.in_min_m) - 1e-6);
}

TEST(Smoother, AKeptHeadStaysWhereItWas) {
  Grid g = room();
  rect(g, 6.0, 3.0, 6.4, 8.0);
  const CostMap cm = build_costmap(g);
  const PlanParams pp = PlanParams::from_geometry(geo());
  Planner pl(cm, pp);
  const auto r = pl.plan({1.5, 5.0}, {8.5, 5.0}, 0.4);
  ASSERT_TRUE(r.ok);
  SmoothParams sp;
  sp.fix_head_m = 1.5;
  const SmoothResult sm = smooth_path(cm, pp, sp, r.path);
  ASSERT_TRUE(sm.ok) << sm.why;
  const auto raw = smooth_detail::resample_arc(r.path, 0.1);
  // the first metre (the curve starts on the fixed control points) follows the input within 2 cm
  for (size_t i = 0; i < 10 && i < sm.path.size() && i < raw.size(); ++i) EXPECT_NEAR(sm.path[i].y, raw[i].y, 0.02) << i;
}

TEST(Smoother, GivesUpInsteadOfBeingWorse) {
  Grid g = room();
  const CostMap cm = build_costmap(g);
  const PlanParams pp = PlanParams::from_geometry(geo());
  EXPECT_FALSE(smooth_path(cm, pp, SmoothParams(), {{1, 1}, {1.2, 1}}).ok);         // too short
  SmoothParams sp;
  sp.max_stretch = 0.5;                                                              // impossible: the curve can't be shorter than the chord
  const auto straight = line({2.0, 5.0}, {8.0, 5.0});
  const SmoothResult r = smooth_path(cm, pp, sp, straight);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.why, "stretch");
  // a path that leaves the map is refused
  EXPECT_FALSE(smooth_path(cm, pp, SmoothParams(), line({2.0, 5.0}, {12.0, 5.0})).ok);
}

TEST(Smoother, PerformanceOnALongPath) {
  Grid g = room(30.0);
  rect(g, 14.9, 0.0, 15.1, 14.0);
  rect(g, 14.9, 16.0, 15.1, 30.0);
  const CostMap cm = build_costmap(g);
  const PlanParams pp = PlanParams::from_geometry(geo());
  Planner pl(cm, pp);
  const auto r = pl.plan({2.0, 2.0}, {28.0, 28.0}, 0.4);
  ASSERT_TRUE(r.ok);
  const auto t0 = std::chrono::steady_clock::now();
  const SmoothResult sm = smooth_path(cm, pp, SmoothParams(), r.path);
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_TRUE(sm.ok) << sm.why;
  EXPECT_LT(ms, 100.0) << "smoothing a " << r.length_m << " m path took " << ms << " ms, " << sm.iters << " iterations";
}

TEST(ClosedLoop, RelayAndSmootherStillCrossTheRoomThroughADoor) {
  Grid world = room();
  rect(world, 4.9, 0.0, 5.1, 4.0);
  rect(world, 4.9, 5.2, 5.1, 10.0);
  const CostMap cm = build_costmap(world);
  const PlanParams pp = PlanParams::from_geometry(geo());
  Follower fol(FollowParams::from(geo(), spd()));
  Pose2 pose{1.5, 1.5, 1.0};
  const Vec2 goal{8.5, 8.5};
  std::vector<Vec2> path;
  Cmd prev;
  double min_fp = 1e9, t_arrive = -1;
  int relayed = 0, replans = 0;
  const double dt = 0.05;
  for (int k = 0; k < 20 * 90; ++k) {
    if (hyp(goal.x - pose.x, goal.y - pose.y) <= 0.4) {
      t_arrive = k * dt;
      break;
    }
    if (k % 20 == 0) {
      const RelayPlan rp = plan_relay(cm, pp, RelayParams(), path, {pose.x, pose.y}, goal, 0.4);
      ASSERT_TRUE(rp.ok) << rp.reason;
      ++replans;
      relayed += rp.relayed;
      SmoothParams sp;
      sp.fix_head_m = rp.relayed ? rp.kept_m : 0.0;
      const SmoothResult sm = smooth_path(cm, pp, sp, rp.path);
      path = sm.ok ? sm.path : rp.path;
      fol.reset();
    }
    const auto pts = blocked_points(world, pose, 4.0);
    const auto o = fol.step(pose, path, pts, &cm, 1.0, prev, dt);
    ASSERT_FALSE(o.blocked) << "blocked at " << pose.x << "," << pose.y;
    prev = o.cmd;
    pose.x += (o.cmd.vx * std::cos(pose.yaw) - o.cmd.vy * std::sin(pose.yaw)) * dt;
    pose.y += (o.cmd.vx * std::sin(pose.yaw) + o.cmd.vy * std::cos(pose.yaw)) * dt;
    pose.yaw += o.cmd.wz * dt;
    min_fp = std::min(min_fp, cm.footprint_clearance(pose.x, pose.y, pose.yaw, pp.fp_offset_m));
  }
  EXPECT_GT(t_arrive, 0) << "did not arrive";
  EXPECT_LT(t_arrive, 60.0);
  EXPECT_GT(relayed, replans / 2) << "most replans should have kept the head";
  EXPECT_GE(min_fp, 0.2) << "the body came too close to a wall";
}

TEST(PlanTrace, DeviationHeadingAndCause) {
  using namespace go2_nav;
  const std::vector<Vec2> a = {{0, 0}, {3, 0}, {6, 0}};
  const std::vector<Vec2> b = {{0, 0}, {1, 0.5}, {3, 0.5}, {6, 0}};
  EXPECT_NEAR(path_deviation(a, a, 3.0), 0.0, 1e-9);
  EXPECT_NEAR(path_deviation(b, a, 3.0), 0.5, 0.02);
  EXPECT_NEAR(path_deviation(a, {}, 3.0), 0.0, 1e-9);
  const PlanDiff d = diff_paths(a, b, 3.0);
  EXPECT_NEAR(d.head_turn_deg, 26.6, 0.5);                       // atan2(0.5, 1)
  EXPECT_NEAR(dist_to_path({1, 2}, a), 2.0, 1e-9);
  EXPECT_NEAR(nearest_point_to_path({{5, 1}, {1, 3}}, a, 3.0), 2.236, 0.001);   // (5,1) is measured to the cut end (3,0); (1,3) is 3 away
  PlanCause c;
  EXPECT_EQ(explain_change(c), "unexplained");
  c.overlay_new_cells = 3;
  c.overlay_near_old_m = 0.4;
  EXPECT_EQ(explain_change(c), "scan_overlay");
  c.pose_jump_m = 0.5;
  EXPECT_EQ(explain_change(c), "pose_jump");
  c.forced = "goal";
  EXPECT_EQ(explain_change(c), "forced:goal");
  PlanCause r;
  r.relay_dropped = true;
  r.relay_why = "off_path";
  EXPECT_EQ(explain_change(r), "relay_dropped:off_path");
}
