Vendored from https://github.com/chvmp/libchamp
Commit: 5572f50532d197c840db2fd283b84f3e1fdb4648
BSD-3-Clause; upstream headers and copyright notices preserved.
ROS1 utils/urdf_loader.h is replaced by include/go2_gait_controller/robot_model.hpp.

Runtime feedback locomotion uses the vendored geometry and Kinematics adapter. Upstream LegController/PhaseGenerator/TrajectoryPlanner are retained as reference, not used by the active controller. State estimation, full-body model, contact state machine and support-force QP are local implementations.
