# go2_v4 · Gazebo 仿真

这是独立于 `go2_v3` 真机工程的纯仿真项目。网页负责启动、操作和停止 Gazebo，支持导入的 `my_map` 固定地图导航，以及当前world中的扫描增量建图/实时导航/前沿探索。

## 启动

在电脑上运行：

```bash
cd /home/an/go2_v4
bash tools/sim_up.sh
```

**每次执行此脚本都会先完整结束上一轮 v4 网页及其 Gazebo/ROS 会话，确认退出后启动新网页。地图、配置和运行记录保留；其他项目的进程不受影响。**

打开 [仿真控制台](http://127.0.0.1:10087/)，选择 `my_map`，点击 **打开 Gazebo 并启动导航**，等待仿真位姿正常，再到「任务」设置目标或航点。没有“允许驱动真狗”的开关。

网页「停止」关闭本次的 `gzserver`、`gzclient` 和 ROS 节点；关闭 Gazebo 窗口也会停止本次仿真。终端 Ctrl-C 关闭网页服务时，会等待这些子进程全部退出。不要在另一个终端重复启动同一仿真。

## 当前内容

- 同一页面内切换「实时导航」工作区，复用原画布与界面样式：手动遥控、已知区域单点导航、自动前沿探索；在线地图从未知开始，默认由Leg-KILO融合仿真点云和IMU定位。扫描红色、近期障碍分黄色（动态，5秒）和青色（静态，90秒），时长可在网页图层面板调。
- 网页目标支持拖动最终朝向；固定图定位条显示真实激光吻合率；固定/实时导航仅显示执行中的蓝色规划路径，不显示红色行走尾迹；接收位姿作显示插值。任务遥控独立为第四个子页签。
- 原 `go2_nav` C++ 规划器：A*、路径接力、B 样条平滑和跟随器，算法源码保持不变。
- 固定地图和实时导航默认使用 **Leg-KILO（LIO：仿真点云+IMU）→ loc_pose** 定位；原算法不改。Gazebo真值仅作物理反馈/评价，不进入正常导航位姿。已验证短程仿真，未验证长距离或真机。
- 场景直接导入 `internship/src/unitree_gazebo/worlds/custom.world`，模型使用 internship 的 Go2 URDF 与 DAE 网格。
- Go2 使用 internship 的真实关节、接触、站立与 reference_trot 对角步态控制器；不再使用平面移动插件。雷达固定在机身上，随实际机身姿态运动。
- Mid360 安装位置、倾角来自 `src/go2_bringup/config/geometry.yaml`。Gazebo 用规则射线近似其 360°、−7°～52°视场，不复现 Livox 非重复扫描花纹、逐点时序或噪声。
- 模拟点云只进入感知节点；网页 3D 点云暂时保持空白，不伪造历史点云。
- 真机 SDK、DDS 凭据工具、板端部署、LIO-SAM 外部接入、旧真机文档已删除。

## 地图与坐标

`maps/my_map/map.yaml` 和 `map.pgm` 是网页使用的地图；`maps/` 根目录保留你传入的原始文件。原图与 internship 的 `my_map.pgm` SHA256 一致，分辨率和原点相同。导入副本将 `free_thresh` 从 0.25 改为 0.196，避免灰度 205 的未知区域被误判为空闲；原始文件没有修改。

该地图与当前 world 配套。换成其他建筑的地图时，还需要提供对应 world 与坐标关系，不能只换图片。

## 编译与检查

```bash
cd /home/an/go2_v4
source ./go2_env.sh
python3 tools/run_bounded.py 300 colcon build --symlink-install --base-paths src --parallel-workers 2
python3 tools/run_bounded.py 420 python3 tools/check_sim.py --gui
GO2_WATCH_STOP_SECONDS=85 python3 tools/run_bounded.py 480 python3 tools/check_realtime.py --slam-exit-probe
```

集成检查使用临时网页端口 10088，验证启动、导航、网页停止、重启、关闭 GUI、关闭网页后的完整收尾。运行前先停止正在使用的 v4 仿真。

隔离配置：ROS 域 78、网页 10087、Gazebo master 11348；不连接真狗，不加载其他工作区的环境。

点击主页顶部「实时导航」，在当前页面内切换控制区（不跳转）。两个模式共用一个仿真，切换前先停止当前会话；实时地图快照存到本次run，不覆盖已有地图。模式选择位于系统卡片下方的原任务页签栏位置，右侧按「1 设置任务 / 2 执行任务」操作，左侧地图列表替换为定位与实时地图信息；窄屏时信息移入右侧。使用方法和限制见[操作](docs/操作.md)。

详细开发记录见 [record.md](record.md)。

详见 [架构](docs/架构.md)、[操作](docs/操作.md)、[本次记录](docs/records/2026-11-11.md)。

普通多段导航：前面被挡/失败/20秒实际无进展的点本圈跳过，仅最后一点检查附近替代落点；不在本圈末尾补跑。循环下一圈重新尝试所有原始点。中间点0.6米内即算经过（不转朝向，只有最后一点转朝向）；多段里代价比最低值高1.5倍或2米持续3秒视为绕大圈，放弃该点（单点/示教不限制绕大圈）；网页运行时显示后续航段虚线和圈内朝向箭头。每点90秒总预算，定位偏低不放宽、丢失仍暂停。运行状态切换与可达性理由写入runs/<run>/route_decisions.jsonl；详见[导航恢复现状](docs/导航恢复现状.md)。

机器人自身受困（mot_nav的blocked/start_blocked）的局部脱困对多点、单点、示教共用同一份实现（go2_task/local_recovery.py）：blocked=后退约0.3米（按loc_pose实际位移判断）→转向；start_blocked=转向→换位置（不后退）；控制链仍单层（TaskExecutor→mot_recover或检查过的临时目标→mot_cmd_mux→safe_gate）。已知问题（待用户决定）：仿真里地图外障碍可使定位降级，sim_bridge在非OK时发零速、狗不动，恢复与导航随之无效；该门槛未改，详见[导航恢复现状](docs/导航恢复现状.md)。
