// @file save_map_cli.cc
// @brief Manual save of the last run (headless build has no "Save Result" button):
//   ros2 run legkilo legkilo_save_map --input_dir <ROOT>/result/<temp>/ --output_dir <new folder> [--voxel 0.05]
#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/slam/tool/headless_save.hpp"

DEFINE_string(input_dir, "", "run result folder (src/legkilo/result/<temp_result_save_folder>/)");
DEFINE_string(output_dir, "", "NEW output folder (it is cleared first)");
DEFINE_double(voxel, 0.05, "voxel size [m] to downsample the global map, <=0 = off");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_input_dir.empty() || FLAGS_output_dir.empty()) {
        std::cerr << "usage: legkilo_save_map --input_dir DIR --output_dir NEW_DIR [--voxel 0.05]\n";
        return 2;
    }
    const bool ok = legkilo::saveRunResult(FLAGS_input_dir, FLAGS_output_dir, FLAGS_voxel);
    std::cout << (ok ? "saved to " : "FAILED, see log: ") << FLAGS_output_dir << std::endl;
    return ok ? 0 : 1;
}
