#pragma once
#include <body_controller/body_controller.h>
#include <kinematics/kinematics.h>
#include "robot_model.hpp"
namespace go2_gait {
// Shared CHAMP geometry/IK adapter. Feedback locomotion lives in champ_controller.
struct Planner { champ::QuadrupedBase base;RobotModel model; };
}
