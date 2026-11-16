# Leg-KILO in this project: what was changed

Upstream: `Leg-KILO-master` (https://github.com/ouguangjun/kilo-map, copied from `~/Desktop/aaa/Leg-KILO-master/legkilo`, 2026-10-21).
Only the **LIO mode** (lidar + IMU) is used on the Go2. Everything else is the unmodified upstream code, so a newer upstream can be diffed against this folder.

| File | Change | Why |
|---|---|---|
| `config/go2_mid360.yaml` | new (was `go2_utlidar.yaml` until 2026-10-30) | Go2 + Livox Mid360 parameters, LIO. Items marked UNVERIFIED in the file still need a check on the real dog. `extrinsic_T/R` and `auto_save_root` are overwritten per run by go2_bringup (`run_dir.write_kilo_config`) from `geometry.yaml` |
| `config/reference_legkilo_go1_velodyne.yaml` | renamed from `legkilo_go1_velodyne.yaml` | kept only because it documents every parameter (KILO mode, Go1); not used |
| `launch_ros2/kilo_go2.launch.py` | new | starts `legkilo_node` with the Go2 config |
| `package.xml` | removed `livox_ros_driver` (ROS 1 package) | not needed on ROS 2 |
| `CMakeLists.txt` | `IRIDESCENCE_VIEWER` default OFF; viewer sources only when ON; `LEGKILO_HEADLESS` + iridescence include dir otherwise; new target `legkilo_save_map` | `libglm-dev` / `libglfw3-dev` are not installed and there is no sudo; headless needs no window |
| `src/viewer/viewer_slam_interface.h` | no-op stub under `LEGKILO_HEADLESS` | backend.cc keeps compiling without the viewer |
| `src/apps/leg_kilo_node_ros2.cc` | null viewer pointer when headless; **auto save on exit** (`auto_save_root`, `auto_save_voxel` in the yaml; skipped when no frame was processed) | the temp result folder is wiped at the next start, and the viewer's "Save Result" button does not exist headless |
| `src/core/slam/tool/headless_save.hpp`, `src/apps/save_map_cli.cc` | new | shared save helper and the manual `legkilo_save_map` tool |
| `src/interface/ros2/ros_interface.cc/.h` | topics `/kilo_odom /kilo_path /kilo_cloud_world /kilo_cloud_body`; frames `kilo_world` / `kilo_base` (was `camera_init` / `base`); lidar + IMU subscriptions **Best Effort**; new yaml key `lidar_time_offset` | project naming; the TF tree of slam_toolbox/bridge already owns `base`; go2_bridge publishes sensor data Best Effort, and a Reliable subscriber (upstream) would never match it |
| `src/core/slam/backend/factor_graph/graph_utils.h` | Ceres version test `>= 2.1` instead of `>= 2` | `ceres::Manifold` exists from Ceres 2.1; Ubuntu 22.04 has 2.0 (`LocalParameterization`) |
| removed | other datasets' configs, `launch_ros2/{legkilo_go1_velodyne,debug}.py`, `test/`, `thirdparty/oneapi-tbb-*.tgz`, ROS 1 launch dir | not used here |

Build: `colcon build --symlink-install --base-paths src` (no conda on PATH; `--base-paths src` keeps colcon out of `unitree_sdk2_python-master`).
Results go to `src/legkilo/result/go2_run/` (wiped at every start) and, on exit, to `<auto_save_root>/kilo_<time>/` (go2_v2: `runs/<run>/kilo/`, see below).

2026-10-23: `config/go2_utlidar.yaml` `time_scale` confirmed on the real dog (seconds); `lidar_time_offset` set to -0.065 (inferred); `auto_save_root` now `/home/an/go2_kilo/maps3d`. Leg-KILO is this project's only localisation: `go2_navigation/scripts/loc_kilo_pose.py` turns `/kilo_odom` into TF `odom->base_footprint`, `/loc_odom`, `/loc_status`. The C++ is unchanged by that.

## go2_v2 (2026-10-25)
Copied from `~/go2_kilo/src/legkilo` without `result/` and `logs/` (runtime output). C++ unchanged. Only `config/go2_utlidar.yaml` changed:
- `lidar_topic` / `imu_topic`: `/bridge_lidar_cloud` / `/bridge_imu` -> `/dev_lidar_cloud` / `/dev_imu` (spec 7.4).
- `auto_save_root`: was hard-coded `/home/an/go2_kilo/maps3d`; now `""` in the source file. `go2_bringup` writes a per-run copy with `auto_save_root: runs/<run>/kilo` and passes it as `config_file`.
- `ROOT_DIR` (compiled in) is this source folder, so `result/` and `logs/` are still written here; `go2_bringup` copies `result/go2_run` and the run's glog files into `runs/<run>/kilo/` when `legkilo_node` exits.
- 2026-10-25: `config/go2_utlidar.yaml` now sets 22 keys Leg-KILO read with defaults (exactly the default values, from its "Key not found" warnings) so the terminal no longer shows 22 warnings per start; `launch_ros2/kilo_go2.launch.py` passes `--colorlogtostderr=false`; `src/common/glog_utils.hpp` prints `GLOG ON/OFF` without the yellow escape codes (user wants a plain white terminal). No behaviour change.

2026-10-30: the unitree lidar was removed and replaced by a Livox Mid360. `config/go2_utlidar.yaml` renamed to `config/go2_mid360.yaml` (also in `launch_ros2/kilo_go2.launch.py`): `lidar_topic` frame is now `mid360_link`, `lidar_time_offset` -0.065 -> 0.0 (the Mid360 stamps the frame start; UNVERIFIED, calibrate with a turn in place), extrinsic from `geometry.yaml`. `time_scale`, `voxel_grid_target_num` for ~20 k points/frame are UNVERIFIED on the board. C++ unchanged (extrinsic direction checked in `KILO.cc`: lidar -> body).
