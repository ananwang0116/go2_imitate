// mot_nav: the motion layer's planner + follower (spec 2 L3), written from scratch (C++), replacing Nav2's planner_server
// and controller_server. task_executor is the only one that gives goals (P2); safe_gate still limits the output (P1).
//
//   in : /mot_nav_goal (go2_msgs/NavGoal, re-sent ~1 Hz as a keep-alive), /per_nav_map (prior + evidence, costmap source),
//        /per_obstacle_scan (fresh obstacles), TF map -> base_footprint
//   out: /mot_nav_cmd_vel (20 Hz while following; mot_cmd_mux picks it behind teleop and recover), /mot_nav_status (10 Hz),
//        /mot_nav_path (each new plan), events [mot] ...
// Rules (logic in include/go2_nav/, tested with gtest):
//   costmap  distance to the nearest blocked cell (map >= 65 = laser-confirmed or hand-drawn wall, or a fresh scan point), exact;
//            hard limit body_half_width + safe_stop_gap, soft cost up to body_half_width + waypoint_clearance; unknown passable at
//            a cost; a saved-map wall nobody has seen this run (60) is passable but expensive and kept at a distance: the laser
//            decides whether it is still there (P4, user decision 2026-10-26)
//   body     two discs (planner.hpp / costmap.hpp footprint_clearance): radius = hard limit, centres half_length - half_width in front of
//            and behind the body centre along the heading (SCAN-Planner's double cylinder, docs/05 section 9)
//   plan     A* on a 2x coarser grid + line-of-sight shortcuts, every plan_period_s, at once on a new goal / blocked path / a hit ahead.
//            Replans of the same goal keep the first relay_m of the old path (relay.hpp), then the path is smoothed by a cubic B-spline
//            that pushes points off walls (smoother.hpp; falls back to the A* path if it would be worse).
//            A PATH goal (taught route) is followed as it is: A* only to join it or to detour a blocked stretch (pathmode.hpp)
//   turn     use_yaw goals: after arriving, turn on the spot to the heading, then report ARRIVED (task_executor no longer asks
//            mot_recover for the final turn)
//   check    service /mot_nav_check: the same planner answers "can the dog reach these waypoints" (checker.hpp), one judge (P2)
//   lookahead every control tick the next ttc_horizon_m of the path is checked (pathcheck.hpp): hit within ttc_replan_s = replan now,
//            speed cap = distance / ttc_slow_s (never below min_move); event PATH_HIT
//   follow   pure pursuit + 107-candidate rollout (4 vx x 3 vy x 9 wz minus zero; 8 when turning first) (1.5 s) against fresh scan points and the map's walls; speeds.yaml limits
//   fail     no_path / goal_blocked / start_blocked after plan_fail_hold_s; blocked after blocked_hold_s (all rollouts collide)
//            -> status FAILED, task_executor decides (recovery ladder)
//   trace    every plan writes a line to <run>/plan_trace.csv; a path that moved (> 0.15 m over its first 3 m, or > 15 deg in the first
//            metre) gives event PLAN_CHANGE with the likeliest cause (plantrace.hpp) and both paths in plan_paths.csv; a failed plan gives PLAN_DIAG
//   safety   goal not refreshed for goal_timeout_s = cancelled; no pose for pose_timeout_s = zero command
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>

#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <go2_msgs/msg/event.hpp>
#include <go2_msgs/msg/nav_goal.hpp>
#include <go2_msgs/msg/nav_status.hpp>
#include <go2_msgs/srv/nav_check.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "go2_nav/config.hpp"
#include "go2_nav/checker.hpp"
#include "go2_nav/costmap.hpp"
#include "go2_nav/events.hpp"
#include "go2_nav/follower.hpp"
#include "go2_nav/plantrace.hpp"
#include "go2_nav/planner.hpp"
#include "go2_nav/relay.hpp"
#include "go2_nav/smoother.hpp"

using namespace go2_nav;
using std::placeholders::_1;

namespace {
// Topic and frame names: the same strings as go2_common/names.py (a python test compares them).
const char* GOAL_TOPIC = "/mot_nav_goal";
const char* STATUS_TOPIC = "/mot_nav_status";
const char* PATH_TOPIC = "/mot_nav_path";
const char* CMD_TOPIC = "/mot_nav_cmd_vel";
const char* NAV_MAP_TOPIC = "/per_nav_map";
const char* SCAN_TOPIC = "/per_obstacle_scan";
const char* EVENT_TOPIC = "/mot_event";
const char* CHECK_SERVICE = "/mot_nav_check";
const char* FRAME_MAP = "map";
const char* FRAME_BASE_FOOTPRINT = "base_footprint";

double yaw_of(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
}  // namespace

enum class State : uint8_t { IDLE = 0, FOLLOWING = 1, ARRIVED = 2, FAILED = 3 };

class MotNav : public rclcpp::Node {
 public:
  MotNav() : Node("mot_nav"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    auto param_or_env = [&](const char* name, const char* env) {
      std::string v = declare_parameter<std::string>(name, "");
      if (v.empty()) {
        const char* e = std::getenv(env);
        if (e) v = e;
      }
      if (v.empty()) throw std::runtime_error(std::string("mot_nav: no ") + name + " (parameter or env " + env + ")");
      return v;
    };
    geometry_ = load_geometry(param_or_env("geometry_file", "GO2_GEOMETRY_FILE"));
    speeds_ = load_speeds(param_or_env("speeds_file", "GO2_SPEEDS_FILE"));
    plan_params_ = PlanParams::from_geometry(geometry_);
    plan_params_.soft_weight = declare_parameter("soft_weight", plan_params_.soft_weight);
    plan_params_.prior_wall_penalty = declare_parameter("prior_wall_penalty", plan_params_.prior_wall_penalty);
    follow_params_ = FollowParams::from(geometry_, speeds_);
    follow_params_.horizon_s = declare_parameter("horizon_s", follow_params_.horizon_s);
    plan_period_s_ = declare_parameter("plan_period_s", 1.0);
    plan_fail_hold_s_ = declare_parameter("plan_fail_hold_s", 2.0);
    blocked_hold_s_ = declare_parameter("blocked_hold_s", 1.5);
    goal_timeout_s_ = declare_parameter("goal_timeout_s", 3.0);
    pose_timeout_s_ = declare_parameter("pose_timeout_s", 0.6);
    scan_keep_s_ = declare_parameter("scan_keep_s", 1.0);
    turn_tol_rad_ = declare_parameter("turn_tol_rad", 0.12);
    turn_timeout_s_ = declare_parameter("turn_timeout_s", 8.0);
    smooth_en_ = declare_parameter("smooth", true);                 // B-spline smoothing of the A* path (smoother.hpp); false = raw A* path
    relay_en_ = declare_parameter("relay", true);                   // keep the near part of the old path when replanning (relay.hpp)
    relay_m_ = declare_parameter("relay_m", 1.0);
    ttc_horizon_m_ = declare_parameter("ttc_horizon_m", 3.0);       // look this far along the path for a hit (pathcheck.hpp)
    ttc_replan_s_ = declare_parameter("ttc_replan_s", 4.0);         // a hit closer than this many seconds: replan now
    ttc_slow_s_ = declare_parameter("ttc_slow_s", 1.5);             // speed cap = distance to the hit / this (never below min_move)
    follower_ = std::make_unique<Follower>(follow_params_);

    const auto cmd_qos = rclcpp::QoS(10).reliable();
    const auto sensor_qos = rclcpp::QoS(5).best_effort();
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    const auto event_qos = rclcpp::QoS(50).reliable().transient_local();
    ctl_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    plan_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    svc_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions io_opts;
    io_opts.callback_group = io_group_;
    goal_sub_ = create_subscription<go2_msgs::msg::NavGoal>(GOAL_TOPIC, cmd_qos, std::bind(&MotNav::on_goal, this, _1), io_opts);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(NAV_MAP_TOPIC, map_qos, std::bind(&MotNav::on_map, this, _1), io_opts);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(SCAN_TOPIC, sensor_qos, std::bind(&MotNav::on_scan, this, _1), io_opts);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(CMD_TOPIC, cmd_qos);
    status_pub_ = create_publisher<go2_msgs::msg::NavStatus>(STATUS_TOPIC, cmd_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(PATH_TOPIC, cmd_qos);
    event_pub_ = create_publisher<go2_msgs::msg::Event>(EVENT_TOPIC, event_qos);
    check_srv_ = create_service<go2_msgs::srv::NavCheck>(
        CHECK_SERVICE, std::bind(&MotNav::on_check, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default,
        svc_group_);
    control_timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&MotNav::control_tick, this), ctl_group_);
    plan_timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&MotNav::plan_tick, this), plan_group_);
    status_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&MotNav::status_tick, this), io_group_);
    event("START", {{"goal_in", fmt(GOAL_TOPIC)}, {"out", fmt(CMD_TOPIC)}, {"hard_m", fmt(plan_params_.hard_m)},
                    {"soft_end_m", fmt(plan_params_.soft_end_m)}, {"vf_mps", fmt(follow_params_.vf)}});
  }

 private:
  struct Goal {
    std::string id;
    bool active = false;
    Vec2 pos;
    double radius = 0.4, scale = 1.0;
    double rx_s = 0;                     // when it was last received (ROS time)
    bool use_yaw = false, follow_path = false, turn_started = false;
    double yaw = 0, turn_t0 = 0;
    std::vector<Vec2> taught;            // path goals: the taught polyline
    size_t taught_hint = 0;
  };
  struct Scan {
    double t;
    std::vector<Vec2> pts;               // map frame
  };

  double now_s() { return get_clock()->now().seconds(); }

  void event(const std::string& name, const Fields& fields) {
    const std::string text = event_text("mot", name, fields);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
    go2_msgs::msg::Event m;
    m.header.stamp = get_clock()->now();
    m.layer = "mot";
    m.name = name;
    for (const auto& kv : fields) {
      diagnostic_msgs::msg::KeyValue k;
      k.key = kv.first;
      std::string v = kv.second;
      if (v.size() >= 2 && v.front() == '"') v = v.substr(1, v.size() - 2);
      k.value = v;
      m.fields.push_back(k);
    }
    event_pub_->publish(m);
    append_event_line(text, now_s());
  }

  bool pose(Pose2& out, double& age_s) {
    try {
      const auto t = tf_buffer_.lookupTransform(FRAME_MAP, FRAME_BASE_FOOTPRINT, tf2::TimePointZero);
      out = {t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)};
      age_s = now_s() - rclcpp::Time(t.header.stamp).seconds();
      return true;
    } catch (const tf2::TransformException&) {
      return false;
    }
  }

  void publish_zero() {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    prev_ = Cmd();
  }

  // a replan outside the 1 Hz rhythm and why (mu_ held); the first reason of several stays
  void want_plan(const char* why) {
    force_plan_ = true;
    if (plan_why_.empty()) plan_why_ = why;
  }

  // <run>/plan_trace.csv (one line per plan) and plan_paths.csv (old and new path whenever a plan changed): where to look when the dog's
  // route changes for no visible reason. One write() on an O_APPEND file, like the events.
  void trace_line(const char* file, const std::string& header, const std::string& line) {
    const char* d = std::getenv("GO2_RUN_DIR");
    if (!d || !*d) return;
    const std::string path = std::string(d) + "/" + file;
    const bool fresh = ::access(path.c_str(), F_OK) != 0;
    const int fd = ::open(path.c_str(), O_WRONLY | O_APPEND | O_CREAT, 0644);
    if (fd < 0) return;
    const std::string text = (fresh ? header + "\n" : std::string()) + line + "\n";
    (void)!::write(fd, text.c_str(), text.size());
    ::close(fd);
  }

  static std::string path_text(const std::vector<Vec2>& p, double step_m = 0.5) {
    std::string out;
    double acc = 1e9;
    for (size_t i = 0; i < p.size(); ++i) {
      if (i > 0) acc += hyp(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
      if (acc >= step_m || i + 1 == p.size()) {
        char b[48];
        std::snprintf(b, sizeof b, "%s%.2f:%.2f", out.empty() ? "" : ";", p[i].x, p[i].y);
        out += b;
        acc = 0;
      }
    }
    return out;
  }

  // ---------- inputs ----------
  void on_goal(const go2_msgs::msg::NavGoal::SharedPtr m) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!m->active) {
      if (goal_.active) event("GOAL_CANCEL", {{"id", fmt(goal_.id)}});
      deactivate();
      return;
    }
    const bool fresh = !goal_.active || m->id != goal_.id;
    goal_.active = true;
    goal_.id = m->id;
    goal_.pos = {m->x, m->y};
    goal_.radius = std::max(0.05, m->radius_m);
    goal_.scale = clampd(m->speed_pct / 100.0, 0.05, 1.0);
    goal_.rx_s = now_s();
    if (fresh) {
      goal_.use_yaw = m->use_yaw;
      goal_.yaw = m->yaw;
      goal_.turn_started = false;
      goal_.follow_path = m->follow_path && m->path_x.size() == m->path_y.size() && m->path_x.size() >= 2;
      if (m->follow_path && !goal_.follow_path) event("GOAL_REJECT", {{"id", fmt(goal_.id)}, {"reason", "bad_path"}});
      goal_.taught.clear();
      goal_.taught_hint = 0;
      if (goal_.follow_path) {
        for (size_t i = 0; i < m->path_x.size(); ++i) goal_.taught.push_back({m->path_x[i], m->path_y[i]});
        goal_.pos = goal_.taught.back();
      }
      path_.clear();
      ++path_version_;
      state_ = State::FOLLOWING;
      reason_ = "planning";
      want_plan("goal");
      plan_fail_since_ = blocked_since_ = -1;
      arrived_reported_ = false;
      event("GOAL", {{"id", fmt(goal_.id)}, {"x_m", fmt(goal_.pos.x)}, {"y_m", fmt(goal_.pos.y)}, {"radius_m", fmt(goal_.radius)},
                     {"path_pts", fmt(static_cast<int>(goal_.taught.size()))}, {"use_yaw", fmt(goal_.use_yaw)}});
    }
  }

  void deactivate() {                    // mu_ held
    const bool moving = state_ == State::FOLLOWING;
    goal_.active = false;
    state_ = State::IDLE;
    reason_.clear();
    path_.clear();
    ++path_version_;
    remaining_ = clearance_ = std::numeric_limits<double>::quiet_NaN();
    if (moving) publish_zero();
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr m) {
    Grid g(static_cast<int>(m->info.width), static_cast<int>(m->info.height), m->info.resolution, m->info.origin.position.x,
           m->info.origin.position.y);
    g.data.assign(m->data.begin(), m->data.end());
    auto cm = std::make_shared<CostMap>(build_costmap(g));
    std::lock_guard<std::mutex> lk(mu_);
    const bool first = !costmap_;
    ++map_n_;
    if (costmap_ && costmap_->grid.w == g.w && costmap_->grid.h == g.h) {       // which cells changed between blocked and not blocked
      const auto& a = costmap_->grid.data;
      for (size_t i = 0; i < g.data.size(); ++i) {
        if ((a[i] >= Grid::BLOCKED) == (g.data[i] >= Grid::BLOCKED)) continue;
        ++map_flips_n_;
        if (map_flips_.size() < 600) map_flips_.push_back(g.center(static_cast<int>(i % g.w), static_cast<int>(i / g.w)));
      }
    }
    costmap_ = cm;
    if (first) event("MAP", {{"width", fmt(static_cast<int>(g.w))}, {"height", fmt(static_cast<int>(g.h))}, {"resolution_m", fmt(g.res)}});
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr m) {
    Pose2 p;
    double age;
    if (!pose(p, age) || age > pose_timeout_s_) return;
    Scan s;
    s.t = now_s();
    const double c = std::cos(p.yaw), sn = std::sin(p.yaw);
    for (size_t i = 0; i < m->ranges.size(); ++i) {
      const double r = m->ranges[i];
      if (!std::isfinite(r) || r < m->range_min || r > m->range_max) continue;
      const double a = m->angle_min + m->angle_increment * static_cast<double>(i);
      const double bx = r * std::cos(a), by = r * std::sin(a);
      s.pts.push_back({p.x + c * bx - sn * by, p.y + sn * bx + c * by});
    }
    std::lock_guard<std::mutex> lk(mu_);
    scans_.push_back(std::move(s));
    while (!scans_.empty() && s.t - scans_.front().t > scan_keep_s_) scans_.pop_front();
  }

  std::vector<Vec2> scan_points(double max_age_s) {     // mu_ held
    std::vector<Vec2> out;
    const double t = now_s();
    for (const Scan& s : scans_)
      if (t - s.t <= max_age_s) out.insert(out.end(), s.pts.begin(), s.pts.end());
    return out;
  }

  // ---------- planning (own thread) ----------
  void plan_tick() {
    Goal goal;
    std::shared_ptr<const CostMap> cm;
    std::vector<Vec2> pts, old_path, flips;
    bool due;
    std::string why;
    int flips_n = 0;
    uint64_t map_n = 0;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (!goal_.active || state_ == State::ARRIVED) return;
      goal = goal_;
      cm = costmap_;
      due = force_plan_ || path_.empty() || now_s() - last_plan_s_ >= plan_period_s_;
      if (!due) return;
      if (!cm) {
        reason_ = "no_map";
        return;
      }
      pts = scan_points(scan_keep_s_);
      old_path = path_;
      force_plan_ = false;
      why = plan_why_;
      plan_why_.clear();
      flips.swap(map_flips_);
      flips_n = map_flips_n_;
      map_flips_n_ = 0;
      map_n = map_n_;
      last_plan_s_ = now_s();
    }
    Pose2 p;
    double age;
    if (!pose(p, age) || age > pose_timeout_s_) {
      std::lock_guard<std::mutex> lk(mu_);
      reason_ = "no_pose";
      want_plan("no_pose");
      return;
    }
    const CostMap overlay = pts.empty() ? *cm : build_costmap(cm->grid, pts);
    struct Out {
      bool ok = false, relayed = false, smoothed = false;
      std::string reason, kind = "astar", smooth_why;
      std::vector<Vec2> path;
      double length_m = 0, ms = 0;
      int expanded = 0;
    } r;
    const auto t_plan0 = std::chrono::steady_clock::now();
    std::string relay_why;
    size_t hint = goal.taught_hint;
    if (goal.follow_path) {
      TaughtPlan tp = plan_on_taught(overlay, plan_params_, TaughtParams(), goal.taught, {p.x, p.y}, hint, pts, geometry_.body_half_width_m,
                                     goal.radius);
      r.ok = tp.ok;
      r.reason = tp.reason;
      r.path = tp.path;
      r.kind = tp.detoured ? "detour" : tp.joined ? "join" : "taught";
      r.length_m = path_length(r.path);
    } else {
      RelayParams rp;
      rp.relay_m = relay_m_;
      if (!relay_en_) old_path.clear();
      const RelayPlan pr = plan_relay(overlay, plan_params_, rp, old_path, {p.x, p.y}, goal.pos, goal.radius);
      r.ok = pr.ok;
      r.reason = pr.reason;
      r.path = pr.path;
      r.length_m = pr.length_m;
      r.expanded = pr.expanded;
      r.relayed = pr.relayed;
      relay_why = pr.relayed ? std::string() : pr.why;
      if (r.ok && smooth_en_) {
        SmoothParams sp;
        sp.fix_head_m = pr.relayed ? pr.kept_m : 0.0;          // a kept head stays as it was
        sp.skip_tail_m = std::max(goal.radius, sp.skip_tail_m);
        const SmoothResult sm = smooth_path(overlay, plan_params_, sp, r.path);
        if (sm.ok) {
          r.path = sm.path;
          r.length_m = path_length(r.path);
          r.smoothed = true;
        } else {
          r.smooth_why = sm.why;               // the A* path stays: smoothing may only make it better
        }
      }
    }
    r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_plan0).count();
    trace_plan(goal, p, why, old_path, r.ok, r.reason, r.kind, r.relayed, relay_why, r.smoothed, r.smooth_why, r.expanded, r.ms, r.path, pts, *cm,
               overlay, flips, flips_n, map_n);
    nav_msgs::msg::Path msg;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (!goal_.active || goal_.id != goal.id) return;           // the goal changed while planning
      goal_.taught_hint = hint;
      if (r.ok) {
        const bool recovered = plan_fail_since_ >= 0 || path_.empty() || r.kind != last_kind_;
        last_kind_ = r.kind;
        path_ = r.path;
        ++path_version_;
        plan_s_ = now_s();
        plan_fail_since_ = -1;
        if (state_ == State::FAILED && fail_reason_is_plan_) state_ = State::FOLLOWING;
        reason_.clear();
        if (recovered)
          event("PLAN", {{"id", fmt(goal.id)}, {"kind", fmt(r.kind)}, {"length_m", fmt(r.length_m)}, {"expanded", fmt(r.expanded)},
                         {"relay", fmt(r.relayed)}, {"smooth", fmt(r.smoothed)}, {"smooth_skip", fmt(r.smooth_why.empty() ? std::string("-") : r.smooth_why)},
                         {"ms", fmt(r.ms)}});
        msg.header.frame_id = FRAME_MAP;
        msg.header.stamp = get_clock()->now();
        for (const Vec2& q : r.path) {
          geometry_msgs::msg::PoseStamped ps;
          ps.header = msg.header;
          ps.pose.position.x = q.x;
          ps.pose.position.y = q.y;
          ps.pose.orientation.w = 1.0;
          msg.poses.push_back(ps);
        }
      } else {
        const double t = now_s();
        if (plan_fail_since_ < 0) {
          plan_fail_since_ = t;
          event("PLAN_FAIL", {{"id", fmt(goal.id)}, {"reason", fmt(r.reason)}});
        }
        reason_ = r.reason;
        if (t - plan_fail_since_ >= plan_fail_hold_s_ && state_ != State::FAILED) {
          state_ = State::FAILED;
          fail_reason_is_plan_ = true;
          path_.clear();
          ++path_version_;
          event("NAV_FAILED", {{"id", fmt(goal.id)}, {"reason", fmt(r.reason)}});
        }
        want_plan("plan_fail");
      }
    }
    if (r.ok) path_pub_->publish(msg);
  }


  // numbers about this plan and what changed since the last one (plantrace.hpp): plan_trace.csv every time, a PLAN_CHANGE event and the
  // two paths in plan_paths.csv when the first 3 m moved by more than 0.15 m or the first metre turned by more than 15 deg.
  void trace_plan(const Goal& goal, const Pose2& p, const std::string& why, const std::vector<Vec2>& old_path, bool ok, const std::string& reason,
                  const std::string& kind, bool relayed, const std::string& relay_why, bool smoothed, const std::string& smooth_why, int expanded,
                  double ms, const std::vector<Vec2>& path, const std::vector<Vec2>& pts, const CostMap& base, const CostMap& overlay,
                  const std::vector<Vec2>& flips, int flips_n, uint64_t map_n) {
    PlanCause c;
    c.forced = why;
    c.relay_dropped = !relayed && old_path.size() >= 2 && goal.follow_path == false && !relay_why.empty() && relay_why != "no_old_path";
    c.relay_why = relay_why;
    c.smooth_skipped = !ok ? false : (!smoothed && smooth_en_);
    // how far the map->odom transform moved since the last plan (the dog's map pose jumps with it)
    double jump = 0, mo_shift = 0, mo_turn = 0;
    try {
      const auto t = tf_buffer_.lookupTransform(FRAME_MAP, "odom", tf2::TimePointZero);
      const Pose2 mo{t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)};
      if (have_map_odom_) {
        mo_shift = hyp(mo.x - map_odom_.x, mo.y - map_odom_.y);
        mo_turn = std::fabs(wrap(mo.yaw - map_odom_.yaw));
        jump = mo_shift + 0.5 * mo_turn;
      }
      map_odom_ = mo;
      have_map_odom_ = true;
    } catch (const tf2::TransformException&) {
    }
    c.pose_jump_m = jump;
    // fresh scan points that block cells the map did not
    std::vector<Vec2> newcells;
    for (const Vec2& q : pts) {
      int i, j;
      base.grid.cell(q.x, q.y, i, j);
      if (base.grid.inside(i, j) && !base.grid.blocked(i, j)) newcells.push_back(q);
    }
    c.overlay_new_cells = static_cast<int>(newcells.size());
    if (old_path.size() >= 2) {
      c.overlay_near_old_m = nearest_point_to_path(newcells, old_path, 1.5);
      c.map_flip_near_old_m = nearest_point_to_path(flips, old_path, 1.5);
    }
    c.map_flips = flips_n;
    double near_dog = std::numeric_limits<double>::infinity();
    for (const Vec2& q : pts) near_dog = std::min(near_dog, hyp(q.x - p.x, q.y - p.y));
    const double scan_new = ok ? nearest_point_to_path(pts, path, 3.0) : std::numeric_limits<double>::quiet_NaN();
    const PlanDiff d = diff_paths(old_path, path, 3.0);
    const bool changed = ok && old_path.size() >= 2 && (d.deviation_m > 0.15 || std::fabs(d.head_turn_deg) > 15.0);
    const std::string cause = !ok ? "plan_failed" : changed ? explain_change(c) : (why.empty() ? "periodic" : "forced:" + why);
    const double t = now_s();
    char b[900];
    std::snprintf(b, sizeof b,
                  "%.3f,%s,%s,%d,%s,%d,%s,%d,%s,%d,%.1f,%.2f,%.2f,%.3f,%.1f,%.3f,%.3f,%.1f,%.3f,%d,%.2f,%.2f,%d,%.2f,%d,%.2f,%llu,%s", t, goal.id.c_str(),
                  why.empty() ? "periodic" : why.c_str(), ok ? 1 : 0, kind.c_str(), relayed ? 1 : 0, relay_why.empty() ? "-" : relay_why.c_str(),
                  smoothed ? 1 : 0, smooth_why.empty() ? "-" : smooth_why.c_str(), expanded, ms, d.length_old_m, d.length_new_m, d.deviation_m,
                  d.head_turn_deg, p.x, p.y, p.yaw * 180.0 / M_PI, jump, static_cast<int>(pts.size()), near_dog, scan_new, c.overlay_new_cells,
                  c.overlay_near_old_m, c.map_flips, c.map_flip_near_old_m, static_cast<unsigned long long>(map_n), cause.c_str());
    trace_line("plan_trace.csv",
               "t,goal,trigger,ok,kind,relay,relay_why,smooth,smooth_skip,expanded,ms,len_old_m,len_new_m,dev_m,head_turn_deg,x,y,yaw_deg,map_odom_jump_m,"
               "scan_pts,scan_near_dog_m,scan_near_newpath_m,overlay_new_cells,overlay_near_old_m,map_flips,map_flip_near_old_m,map_n,cause",
               b);
    if (!ok) {
      event("PLAN_DIAG", {{"id", fmt(goal.id)}, {"reason", fmt(reason)}, {"x_m", fmt(p.x)}, {"y_m", fmt(p.y)},
                          {"start_clear_m", fmt(overlay.clearance(p.x, p.y))}, {"goal_clear_m", fmt(overlay.clearance(goal.pos.x, goal.pos.y))},
                          {"scan_pts", fmt(static_cast<int>(pts.size()))}, {"scan_near_dog_m", fmt(near_dog)},
                          {"overlay_new_cells", fmt(c.overlay_new_cells)}, {"map_flips", fmt(c.map_flips)}});
    } else if (changed) {
      event("PLAN_CHANGE", {{"id", fmt(goal.id)}, {"cause", fmt(cause)}, {"dev_m", fmt(d.deviation_m)}, {"head_turn_deg", fmt(d.head_turn_deg)},
                            {"len_old_m", fmt(d.length_old_m)}, {"len_new_m", fmt(d.length_new_m)}, {"trigger", fmt(why.empty() ? std::string("periodic") : why)},
                            {"relay", fmt(relayed)}, {"relay_why", fmt(relay_why.empty() ? std::string("-") : relay_why)},
                            {"overlay_new_cells", fmt(c.overlay_new_cells)}, {"overlay_near_old_m", fmt(c.overlay_near_old_m)},
                            {"map_flips", fmt(c.map_flips)}, {"map_flip_near_old_m", fmt(c.map_flip_near_old_m)},
                            {"map_odom_jump_m", fmt(jump)}, {"scan_near_dog_m", fmt(near_dog)}});
      char pb[64];
      std::snprintf(pb, sizeof pb, "%.3f,%s,%s,", t, goal.id.c_str(), cause.c_str());
      trace_line("plan_paths.csv", "t,goal,cause,old_path,new_path", std::string(pb) + path_text(old_path) + "," + path_text(path));
    }
  }

  // ---------- control (20 Hz) ----------
  void control_tick() {
    std::lock_guard<std::mutex> lk(mu_);
    const double t = now_s();
    if (goal_.active && t - goal_.rx_s > goal_timeout_s_) {
      event("GOAL_TIMEOUT", {{"id", fmt(goal_.id)}, {"after_s", fmt(goal_timeout_s_)}});
      deactivate();
      return;
    }
    if (!goal_.active) return;
    if (state_ == State::ARRIVED) return;
    Pose2 p;
    double age;
    if (!pose(p, age) || age > pose_timeout_s_) {
      if (reason_ != "no_pose") event("NO_POSE", {{"age_s", fmt(age)}});
      reason_ = "no_pose";
      publish_zero();
      return;
    }
    if (reason_ == "no_pose") reason_.clear();
    if (goal_.turn_started || hyp(goal_.pos.x - p.x, goal_.pos.y - p.y) <= goal_.radius) {
      if (goal_.use_yaw) {
        const double err = wrap(goal_.yaw - p.yaw);
        if (!goal_.turn_started) {
          goal_.turn_started = true;
          goal_.turn_t0 = t;
          reason_ = "turning";
          event("FINAL_TURN", {{"id", fmt(goal_.id)}, {"yaw_deg", fmt(goal_.yaw * 180.0 / M_PI)}, {"err_deg", fmt(err * 180.0 / M_PI)}});
        }
        const bool timeout = t - goal_.turn_t0 > turn_timeout_s_;
        if (std::fabs(err) > turn_tol_rad_ && !timeout) {
          geometry_msgs::msg::Twist m;
          m.angular.z = turn_command(err, speeds_.turn_radps, goal_.scale, prev_.wz, 0.05, follow_params_.ang_accel, turn_tol_rad_);
          prev_ = {0, 0, m.angular.z};
          cmd_pub_->publish(m);
          return;
        }
        if (timeout) event("FINAL_TURN_TIMEOUT", {{"id", fmt(goal_.id)}, {"err_deg", fmt(err * 180.0 / M_PI)}});
      }
      state_ = State::ARRIVED;
      reason_.clear();
      publish_zero();
      if (!arrived_reported_) {
        arrived_reported_ = true;
        event("ARRIVED", {{"id", fmt(goal_.id)}, {"x_m", fmt(p.x)}, {"y_m", fmt(p.y)}});
      }
      return;
    }
    if (state_ == State::FAILED || path_.size() < 2) {
      if (prev_.vx != 0 || prev_.vy != 0 || prev_.wz != 0) publish_zero();
      return;
    }
    if (path_version_ != followed_version_) {
      follower_->reset();
      followed_version_ = path_version_;
    }
    const std::vector<Vec2> pts = scan_points(0.3);
    const FollowOut o = follower_->step(p, path_, pts, costmap_.get(), goal_.scale, prev_, 0.05);
    remaining_ = o.remaining_m;
    clearance_ = o.clearance_m;
    if (o.blocked) {
      if (blocked_since_ < 0) {
        blocked_since_ = t;
        want_plan("blocked");
        event("BLOCKED", {{"id", fmt(goal_.id)}, {"x_m", fmt(p.x)}, {"y_m", fmt(p.y)}});
      }
      publish_zero();
      if (t - blocked_since_ >= blocked_hold_s_) {
        state_ = State::FAILED;
        reason_ = "blocked";
        fail_reason_is_plan_ = false;
        event("NAV_FAILED", {{"id", fmt(goal_.id)}, {"reason", "blocked"}});
      }
      return;
    }
    if (blocked_since_ >= 0) {
      blocked_since_ = -1;
      event("UNBLOCKED", {{"id", fmt(goal_.id)}});
    }
    // look ahead along the path: the distance to the first hit becomes a time; soon = plan again now, close = slow down
    Cmd cmd = o.cmd;
    const PathHit hit = path_hit(p, pts);
    if (hit.hit) {
      const double v_ref = std::max(std::hypot(prev_.vx, prev_.vy), 0.3);
      const double ttc = hit.dist_m / v_ref;
      if (ttc < ttc_replan_s_ && t - hint_replan_s_ >= 0.4) {
        hint_replan_s_ = t;
        want_plan("hit");
      }
      const double vcap = std::max(follow_params_.min_move, hit.dist_m / ttc_slow_s_);
      if (cmd.vx > vcap) cmd.vx = vcap;
      if (!hit_reported_ || t - hit_event_s_ > 2.0) {
        hit_event_s_ = t;
        event("PATH_HIT", {{"id", fmt(goal_.id)}, {"dist_m", fmt(hit.dist_m)}, {"ttc_s", fmt(ttc)}, {"cap_mps", fmt(vcap)}});
      }
      hit_reported_ = true;
    } else {
      hit_reported_ = false;
    }
    prev_ = cmd;
    geometry_msgs::msg::Twist m;
    m.linear.x = cmd.vx;
    m.linear.y = cmd.vy;
    m.angular.z = cmd.wz;
    cmd_pub_->publish(m);
  }

  // where, if at all, the body would hit something on the next ttc_horizon_m of the path (pathcheck.hpp): the map's walls with the
  // two-point footprint and the fresh scan points. mu_ held.
  PathHit path_hit(const Pose2& p, const std::vector<Vec2>& pts) {
    PathCheckParams pc;
    pc.horizon_m = ttc_horizon_m_;
    pc.near_m = plan_params_.start_relax_m;
    pc.scan_r_m = geometry_.body_half_width_m + 0.05;
    pc.tail_free_m = goal_.radius;
    return first_hit(costmap_.get(), plan_params_, pc, path_, {p.x, p.y}, pts);
  }

  // ---------- service: reachability asked of the same costmap and planner ----------
  void on_check(const std::shared_ptr<go2_msgs::srv::NavCheck::Request> req, std::shared_ptr<go2_msgs::srv::NavCheck::Response> res) {
    std::shared_ptr<const CostMap> cm;
    {
      std::lock_guard<std::mutex> lk(mu_);
      cm = costmap_;
    }
    res->available = cm != nullptr;
    if (!cm) return;
    CheckIn in;
    const size_t n = std::min(req->x.size(), req->y.size());
    for (size_t k = 0; k < n; ++k) {
      in.pos.push_back({req->x[k], req->y[k]});
      in.radius.push_back(k < req->radius_m.size() ? req->radius_m[k] : geometry_.arrive_radius_m);
      std::vector<Vec2> path;
      if (req->path_off.size() == n + 1) {
        for (uint32_t q = req->path_off[k]; q < req->path_off[k + 1] && q < req->path_x.size() && q < req->path_y.size(); ++q)
          path.push_back({req->path_x[q], req->path_y[q]});
      }
      in.paths.push_back(path);
    }
    in.use_start = req->use_start;
    in.start = {req->start_x, req->start_y};
    const CheckOut out = check_waypoints(*cm, plan_params_, geometry_, in);
    res->leg_path_off.push_back(0);
    for (size_t k = 0; k < n; ++k) {
      res->ok.push_back(out.ok[k]);
      res->reason.push_back(out.reason[k]);
      res->leg_m.push_back(out.leg_m[k]);
      for (const Vec2& q : out.leg_path[k]) {
        res->leg_path_x.push_back(q.x);
        res->leg_path_y.push_back(q.y);
      }
      res->leg_path_off.push_back(static_cast<uint32_t>(res->leg_path_x.size()));
    }
  }

  // ---------- status (10 Hz) ----------
  void status_tick() {
    go2_msgs::msg::NavStatus s;
    s.header.stamp = get_clock()->now();
    std::lock_guard<std::mutex> lk(mu_);
    s.id = goal_.active ? goal_.id : "";
    s.state = static_cast<uint8_t>(state_);
    s.reason = reason_;
    s.remaining_m = path_.size() > 1 ? remaining_ : std::numeric_limits<double>::quiet_NaN();
    s.clearance_m = clearance_;
    s.plan_age_s = path_.empty() ? std::numeric_limits<double>::quiet_NaN() : now_s() - plan_s_;
    status_pub_->publish(s);
  }

  // parameters and logic objects
  Geometry geometry_{};
  Speeds speeds_{};
  PlanParams plan_params_;
  FollowParams follow_params_;
  bool smooth_en_ = true, relay_en_ = true, hit_reported_ = false;
  double relay_m_ = 1.0, ttc_horizon_m_ = 3.0, ttc_replan_s_ = 4.0, ttc_slow_s_ = 1.5, hit_event_s_ = 0;
  double plan_period_s_, plan_fail_hold_s_, blocked_hold_s_, goal_timeout_s_, pose_timeout_s_, scan_keep_s_, turn_tol_rad_, turn_timeout_s_;
  std::unique_ptr<Follower> follower_;

  // shared state (mu_)
  std::mutex mu_;
  Goal goal_;
  std::shared_ptr<const CostMap> costmap_;
  std::deque<Scan> scans_;
  std::vector<Vec2> path_;
  uint64_t path_version_ = 0, followed_version_ = 0;
  State state_ = State::IDLE;
  std::string reason_;
  bool force_plan_ = false, arrived_reported_ = false, fail_reason_is_plan_ = false;
  std::string last_kind_, plan_why_;
  uint64_t map_n_ = 0;
  int map_flips_n_ = 0;
  std::vector<Vec2> map_flips_;
  Pose2 map_odom_;
  bool have_map_odom_ = false;
  double last_plan_s_ = 0, plan_s_ = 0, plan_fail_since_ = -1, blocked_since_ = -1, hint_replan_s_ = 0;
  double remaining_ = std::numeric_limits<double>::quiet_NaN(), clearance_ = std::numeric_limits<double>::quiet_NaN();
  Cmd prev_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::CallbackGroup::SharedPtr ctl_group_, plan_group_, io_group_, svc_group_;
  rclcpp::Service<go2_msgs::srv::NavCheck>::SharedPtr check_srv_;
  rclcpp::Subscription<go2_msgs::msg::NavGoal>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<go2_msgs::msg::NavStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<go2_msgs::msg::Event>::SharedPtr event_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_, plan_timer_, status_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MotNav>();
    rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 3);
    ex.add_node(node);
    ex.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("mot_nav"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
