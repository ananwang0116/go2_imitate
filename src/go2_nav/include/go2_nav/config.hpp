// geometry.yaml and speeds.yaml: the only places that hold body sizes, distances and speed limits (spec 3, speeds.yaml).
// Read with yaml-cpp, same keys as go2_common/geometry.py and speeds.py; a missing or unknown key is an error.
// The mid360_* mount keys are only checked for presence here: the navigation does not use the lidar pose.
#pragma once
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace go2_nav {

struct Geometry {
  double body_half_length_m, body_half_width_m, loc_sigma_m, brake_decel_mps2, brake_reaction_s, safe_stop_gap_m, avoid_gap_m,
      waypoint_margin_m, waypoint_clearance_m, arrive_radius_m;
};
struct Speeds {
  double forward_mps, back_mps, side_mps, turn_radps, manual_side_mps;
};

inline void expect_keys(const YAML::Node& n, std::initializer_list<const char*> keys, const std::string& path) {
  for (const char* k : keys)
    if (!n[k]) throw std::runtime_error(path + ": missing key " + k);
  for (auto it = n.begin(); it != n.end(); ++it) {
    bool known = false;
    for (const char* k : keys) known = known || it->first.as<std::string>() == k;
    if (!known) throw std::runtime_error(path + ": unknown key " + it->first.as<std::string>());
  }
}

inline Geometry load_geometry(const std::string& path) {
  YAML::Node n = YAML::LoadFile(path);
  expect_keys(n, {"body_half_length_m", "body_half_width_m", "loc_sigma_m", "brake_decel_mps2", "brake_reaction_s",
                  "safe_stop_gap_m", "avoid_gap_m", "waypoint_margin_m", "waypoint_clearance_m", "arrive_radius_m",
                  "mid360_x_m", "mid360_y_m", "mid360_z_m", "mid360_roll_deg", "mid360_pitch_deg", "mid360_yaw_deg"}, path);
  Geometry g;
  g.body_half_length_m = n["body_half_length_m"].as<double>();
  g.body_half_width_m = n["body_half_width_m"].as<double>();
  g.loc_sigma_m = n["loc_sigma_m"].as<double>();
  g.brake_decel_mps2 = n["brake_decel_mps2"].as<double>();
  g.brake_reaction_s = n["brake_reaction_s"].as<double>();
  g.safe_stop_gap_m = n["safe_stop_gap_m"].as<double>();
  g.avoid_gap_m = n["avoid_gap_m"].as<double>();
  g.waypoint_margin_m = n["waypoint_margin_m"].as<double>();
  g.waypoint_clearance_m = n["waypoint_clearance_m"].as<double>();
  g.arrive_radius_m = n["arrive_radius_m"].as<double>();
  if (g.body_half_length_m <= 0 || g.body_half_width_m <= 0 || g.safe_stop_gap_m <= 0)
    throw std::runtime_error(path + ": body size and safe_stop_gap_m must be positive");
  return g;
}

inline Speeds load_speeds(const std::string& path) {
  YAML::Node n = YAML::LoadFile(path);
  expect_keys(n, {"forward_mps", "back_mps", "side_mps", "turn_radps", "manual_side_mps"}, path);
  Speeds s{n["forward_mps"].as<double>(), n["back_mps"].as<double>(), n["side_mps"].as<double>(), n["turn_radps"].as<double>(),
           n["manual_side_mps"].as<double>()};
  for (double v : {s.forward_mps, s.back_mps, s.side_mps, s.turn_radps, s.manual_side_mps})
    if (!(v > 0 && v <= 1.0)) throw std::runtime_error(path + ": speeds must be in (0, 1.0]");
  return s;
}

}  // namespace go2_nav
