// @file headless_save.hpp
// @brief Save the run's result (single global PCD + TUM trajectories) without the viewer's "Save Result" dialog.
// Used by leg_kilo_node_ros2 (auto save on exit) and legkilo_save_map (manual).
#ifndef LEGKILO_HEADLESS_SAVE_HPP
#define LEGKILO_HEADLESS_SAVE_HPP

#include <string>

#include <glog/logging.h>

#include "core/slam/tool/slam_result_saver.h"
#include "guik/progress_interface.hpp"

namespace legkilo {

struct LogProgress : public guik::ProgressInterface {
    void set_text(const std::string& text) override { LOG(INFO) << "[save] " << text; }
};

// input_folder: the run's temp result folder (<ROOT_DIR>/result/<temp_result_save_folder>/).
// output_folder: must be a NEW folder; the saver clears it first.
inline bool saveRunResult(const std::string& input_folder, const std::string& output_folder, double voxel) {
    SLAMResultSaver saver;
    saver.setInputFolder(input_folder);
    saver.setResultFolder(output_folder);
    saver.setTrajectoryEnable(CoordType::IMU, TrajectoryFormat::TUM, TrajectoryType::ALL);
    saver.setGlobalMapEnable(CoordType::IMU, GlobalMapSizeType::SINGLE, 50.0, voxel > 0.0, voxel);
    LogProgress progress;
    return saver.run(progress);
}

}  // namespace legkilo
#endif  // LEGKILO_HEADLESS_SAVE_HPP
