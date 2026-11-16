// SPDX-License-Identifier: MIT
// @file leg_kilo_node_ros2.cc
// @brief Main node for Leg KILO SLAM system (ROS2).
#include <csignal>
#include <fstream>
#include <ctime>
#include <memory>
#include <vector>

#include <unistd.h>

#include <pcl/console/print.h>
#include <rclcpp/rclcpp.hpp>

#include "common/glog_utils.hpp"
#include "common/timer_utils.hpp"
#include "common/yaml_helper.hpp"
#include "core/slam/tool/headless_save.hpp"
#include "interface/ros2/ros_interface.h"
#include "viewer/viewer_slam_interface.h"

DEFINE_string(config_file, "config/leg_fusion.yaml", "Path to the YAML file");

namespace {
volatile std::sig_atomic_t shutdown_requested = 0;
}

void sigHandle(int sig) { shutdown_requested = 1; }

int main(int argc, char** argv) {
    auto non_ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    std::vector<char*> gflags_argv;
    gflags_argv.reserve(non_ros_args.size());
    for (auto& arg : non_ros_args) { gflags_argv.push_back(arg.data()); }

    auto node = std::make_shared<rclcpp::Node>("legkilo", "legkilo");

    signal(SIGINT, sigHandle);

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    std::unique_ptr<legkilo::Logging> logging =
        std::make_unique<legkilo::Logging>(static_cast<int>(gflags_argv.size()), gflags_argv.data(), "logs", 20);

    if (FLAGS_config_file.empty()) {
        LOG(ERROR) << "YAML configuration file path not provided. Use --config_file=<path>.";
        rclcpp::shutdown();
        return -1;
    }

#ifdef LEGKILO_HEADLESS
    // No window: the backend skips every viewer call when it gets a null pointer.
    legkilo::ViewerSlamInterface* viewer_interface = nullptr;
#else
    auto viewer_interface_owner = std::make_unique<legkilo::ViewerSlamInterface>(FLAGS_config_file);
    viewer_interface_owner->start();
    legkilo::ViewerSlamInterface* viewer_interface = viewer_interface_owner.get();
#endif

    std::unique_ptr<legkilo::RosInterface> ros_interface_node =
        std::make_unique<legkilo::RosInterface>(node, viewer_interface);
    ros_interface_node->init(FLAGS_config_file);

    LOG(INFO) << "Leg KILO Node Starts";

    rclcpp::WallRate running_rate(5000.0);
    rclcpp::WallRate idle_rate(10.0);
    while (rclcpp::ok() && !shutdown_requested && !(viewer_interface && viewer_interface->shouldTerminate())) {
        if (!ros_interface_node->isSlamStopped()) { ros_interface_node->run(); }

        if (!ros_interface_node->isSlamStopped() && viewer_interface && viewer_interface->consumeSLAMTerminateRequest()) {
            ros_interface_node->stopSlam();
            LOG(INFO) << "SLAM stopped by viewer request";
        }

        ros_interface_node->isSlamStopped() ? idle_rate.sleep() : running_rate.sleep();
    }

    ros_interface_node->stop();
    if (viewer_interface) viewer_interface->stop();

    ros_interface_node.reset();   // stops the backend: submaps and records are flushed to result/<temp>/ here
#ifndef LEGKILO_HEADLESS
    viewer_interface_owner.reset();
#endif
    rclcpp::shutdown();

    // The temp result folder is wiped at the next start, so write the PCD map now (auto_save_root empty = off).
    {
        legkilo::YamlHelper yaml(FLAGS_config_file);
        const std::string root = yaml.get<std::string>("auto_save_root", "");
        const std::string input = std::string(ROOT_DIR) + "result/" + yaml.get<std::string>("temp_result_save_folder", "temp") + "/";
        // a run that never processed a frame has only the header line in frontend_frames.csv: nothing to save
        std::ifstream frames_csv(input + "frontend_frames.csv");
        std::string header, first_row;
        const bool has_frames = frames_csv && std::getline(frames_csv, header) && std::getline(frames_csv, first_row);
        if (!root.empty() && !has_frames) { LOG(INFO) << "Auto save skipped: no frames were processed"; }
        if (!root.empty() && has_frames) {
            const std::time_t t = std::time(nullptr);
            char stamp[32];
            std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));
            const std::string output = root + "/kilo_" + stamp;
            LOG(INFO) << "Auto save: " << input << " -> " << output;
            const bool ok = legkilo::saveRunResult(input, output, yaml.get<double>("auto_save_voxel", 0.05));
            LOG(INFO) << (ok ? "Auto save done: " : "Auto save FAILED: ") << output;
        }
    }

    LOG(INFO) << "RosInterface destroyed";
    LOG(INFO) << "Leg KILO Node Ends";
    legkilo::Timer::logAllAverTime();
    logging->flushLogFiles();
    return 0;
}
