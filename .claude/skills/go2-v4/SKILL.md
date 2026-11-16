---
name: go2-v4
description: Work on go2_v4 Gazebo simulation, including coding, analysis, build/test, managed startup, watchdogs, records and document updates. Read before each project task and before declaring completion.
---
# go2_v4 simulation rules

- User authorized a simulation fork: Gazebo and simulated motion are explicitly required.
- /home/an/go2_v3 is a read-only reference. Modify and build only go2_v4. Exception (user, 2026-11-13): a one-time, explicitly requested port of v4 logic into go2_v3 (done: see record.md; backup runs/_backups/go2_v3_before_v4_port_20261113.tar.gz; desk-tested only, never started there). v3 stays read-only otherwise.
- Use ROS_DOMAIN_ID=78 and GAZEBO_MASTER_URI=http://127.0.0.1:11348. Never use real DDS credentials or board startup here.
- Preserve the imported go2_nav planner and Leg-KILO algorithms. Simulation integration belongs in go2_sim and the web adapter.
- Since the user request on 2026-11-11, both fixed-map and realtime modes default to Leg-KILO (LIO: simulated point cloud + body IMU) → loc_pose localisation. Ground-truth mode requires explicit diagnostic selection; never silently fall back. Gazebo truth is allowed only for plant/device fault feedback and evaluation. Claim only the tested short simulation scope, never real-robot or long-term SLAM validation.
- No synthetic point cloud in the web 3D view. Keep it empty until real simulated 3D sensor integration is implemented.
- Use bounded commands, progress/health checks and owned process groups. Never kill by process name or path pattern.
- Keep commands on loopback. Shut down only processes started by this project. Report every minute during long work.
- Record changes, checks and limitations in docs/records. Imported v3 rules are historical reference; these simulation rules override conflicting real-robot restrictions.

- 用户要求维护根目录 record.md，按“问题”组织，而不是按时间流水账（2026-11-11更新）：每个问题一节，依次写①发现了什么问题（现象+证据/run路径+原因，原因未知就写未知）②尝试过哪些方式、各自怎么处理、结果如何（含失败、放弃和为什么）③最终怎么解决（采用的方案与理由，哪些文件/规则变了）④验证与未验证（区分离线逻辑/Gazebo物理/推断/未测）⑤遗留限制。同一问题的多次尝试写在同一节内，不为每次尝试另开新节；新问题另开新节。
- 使用 internship 的 reference_trot 关节/接触动力学；禁止把平面移动或腿部动画描述为真实步态。传感器固定在base，姿态来自实际body运动。
- 启动必须按spawn成功→控制器依次加载→站立轨迹→trot交接/反馈检查的依赖顺序推进，失败立即停止且不继续后续阶段。

- 用户要求每次sim_up先结束上一轮v4网页/仿真并确认端口释放，再开新网页；仅清理可验证归属的会话，不删除地图与run，不按名字批量杀进程。


## Automatically enforced workflow (inherited from v3 records 50–52)

These are standing user requirements; do not wait for the user to repeat them.
- Before each task/phase, set its deadline, progress/health probes and failure cleanup. Short reads/analysis must use bounded commands (explicit tool timeout/yield and output limits); builds/tests/simulation checks must run under `tools/run_bounded.py <seconds> <command...>` or an equivalent owned supervisor. A bare timeout or a successful process spawn is not a readiness/health check.
- Keep watchdog coverage when changing phases. The outer deadline must allow the inner startup, test and graceful-stop budgets. A persistent user console uses StackManager's ownership, startup deadline and core-exit monitors, plus command/feedback watchdogs; do not put an arbitrary expiry on a user-owned running console.
- Before simulation, check existing domain78/session/ports; use one owned stack only. READY requires staged spawn/controllers/standing/trot, valid pose, map, scan and task feedback. On failure stop and verify only this run's PID/start-time/token-owned processes. Never clean another workspace or kill by name.
- During checks, report health/progress every 30–60 seconds; source/node exit fails immediately, sustained missing/stale required pose/sensors for >3 seconds stops the autonomous check. Include transient faults and recovery in the record. Web autonomous control must pause/cancel on client lease loss; manual motion remains subject to command/feedback expiry.
- `record.md` and `docs/records/YYYY-MM-DD.md` are mandatory for every project session, including failed attempts, and use the problem-oriented layout above: 问题 → 尝试/处理方式 → 最终解决 → 验证/未验证 → 遗留。Each section also carries goal/environment, evidence/run path, watchdog budgets and process/port/lock cleanup. Preserve historical evidence (never delete earlier sections; earlier chronological entries stay as they are); distinguish offline logic, Gazebo physics, inference and untested claims. A problem that is still open stays open in its section; when later resolved, update the same section instead of appending a new one.
- Before saying done, update affected current instructions/specs (`README.md`, `docs/架构.md`, `docs/操作.md`, node docstrings and this skill), search for stale commands/modes/interfaces, and record watchdog exit plus process/port/lock cleanup. Appending a record alone is insufficient when the current docs are wrong.
- Planning reachability is judged by the original `/mot_nav_check`; exploration may rank frontier candidates but must ask that service before dispatch. Use the existing task executor as the sole publisher of motion navigation goals. Log goal/mode changes and their causes.
- Accumulated maps live in a specified frame: clear/invalidate them on a frame-origin reset; never update old cells using a latest-pose fallback for a scan whose timestamp TF is unavailable. Missing radar returns mean unknown, not free.

Reference provenance: read-only `/home/an/go2_v3/.claude/skills/go2-v3/SKILL.md`, Part A ground rules/watchdog/READY and Part B records/docs, checked 2026-11-11. v3 real-DDS, board, network and real-robot startup commands do not apply to this isolated simulation fork.

- Follow `docs/架构.md`'s single-owner responsibility table (adapted from v3 docs): mapper only owns /map; per_evidence alone owns fused /per_nav_map; exploration proposes candidates only; task_executor alone issues motion navigation goals; mot_nav alone plans/follows; mot_cmd_mux alone chooses a velocity source; safe_gate alone limits automatic commands. Do not add parallel control paths when adding a UI or a mode.

- Leg-KILO inputs are gated until `SIM_GAIT_READY`; warm >=50 actual IMU samples before forwarding a cloud. Gazebo rays are instantaneous, so point `time=0` is intentional; never invent a 0.1s rolling scan. Sensor stamps and raw lidar axes must be retained, with extrinsics generated from geometry.yaml. Body TF in SLAM mode is estimated from /kilo_odom, not /sim/model_states. Initial floor height is an explicit configured .35m reference, not measured navigation truth.
- SLAM stop budgets nest: Leg-KILO export/grace40s < managed escalation55s < console wait65s < launcher/test wait75s; use outer watchdog stop grace85s for simulation checks. Save per-run frontend/backend trajectories, map and runtime/glog before next startup wipes temporary results.

- When changing localisation/UI modes, audit inherited `display:none`, disabled/lock and colour-inheritance rules as well as API handlers. Verify the initial-pose control and workspace entries in the browser at normal and narrow widths; backend availability alone does not prove an operation is visible/useable.

- Realtime is an in-place workspace in the original index UI: no redirect, second design/template or iframe. Session identity, not mode alone, owns all web TF/state/image caches. Reset them on every restart; reject future TF after simulated-clock rollback and discard work/images from older sessions. Initial-pose feedback must correlate the owner's request-stamped event and delivered TF before showing completion; a healthy LIO status alone is not an initial-pose acknowledgement.
- When removing/renaming packaged web resources, clean only that package's generated build/install artifacts before rebuilding; stale setuptools/symlink resource lists can still reference deleted files. Never remove maps or runs as build cleanup.

- 固定图定位分数使用loc_map_check（扫描自身时戳TF，最多1秒待匹配队列），不得用最新位姿回退；网页分数3秒过期，实时模式不伪造静态图吻合率。可选目标yaw必须贯通到原task_executor；无yaw保持全零四元数约定。
- 实际轨迹分会话并在初始定位清空；显示插值只能在已接收位姿之间、不得外推或修改控制位姿。实时模式选择放系统卡片下方、原任务子页签的位置，宽屏地图信息替换左地图列表；窄屏需保留信息入口。
- sim_startup单进程依次加载/配置原控制器并复用原StandCommand；不可并行控制依赖、删5秒站立或放宽物理/SLAM就绪门槛。切换colcon普通/符号链接安装后重新source go2_env.sh，再运行已安装入口。

- 扫描根因修复应落在唯一per_scan公共处理内，不复制仿真扫描算法：原点云必须等待自身时戳机身TF；每帧地面拟合不可无条件复用上一运动机身坐标的plane。零拟合使用当前base_footprint零参考，当前验证范围为水平地面。
- 用户明确导航仅需执行中的蓝色规划路径，到达即消失，不要再加红色行走尾迹；建图/示教原线保留。任务页遥控是第四子页签，复用原面板/控制函数，仅移动位置。

- 近期障碍（2026-11-12起取代统一30秒）：同一格连续被占据<3秒=动态（网页黄色/证据图99），最后命中后5秒过期；≥3秒=静态（青色/100），90秒过期；视野内被射线穿过会很快清除；99和100都阻挡规划。由唯一per_evidence公共逻辑维护；时长可在网页图层实时调（经/per_evidence_ttl_cmd，只本次运行有效），固定/实时launch参数（EVIDENCE_TTL）、网页图例保持一致，持久地图不按TTL删除。task_executor每点90秒非暂停时间，DEGRADED不得豁免、LOST撤运动并暂停。普通多点任务中间点被挡/失败/20秒实际空间无进展（近20秒内离当前位置始终不足0.4米；2026-11-12由“新位置距已计入位置0.3米”改为滑动窗口，单点/示教同用）则本圈跳过；中间点到达半径0.6米且不转朝向（仅最后一点转朝向）；绕大圈用原规划器代价上限（best×1.5或+2米持续3秒→detour_too_long，仅多段，用/mot_nav_check的leg_m，no_path/goal_blocked另算；单点/示教不限制绕大圈）；单点/示教：不限整段用时，只限“问题处理”真实累计时间（WaypointRun.trouble_s，任何事件都不重置；满90秒停一次提示后继续，满180秒放弃并提示）；no_path/no_progress=后退→转向，goal_blocked=逐个检查的替代落点（示教除外，无起点型特殊处理、不弹窗）；网页后续航段预览仅来自原/mot_nav_check的leg_path，仅显示，只有原始末点经原/mot_nav_check验证附近候选后尝试（最多5次）；不在圈末补跑，循环下一圈重新尝试所有原始点，替代不得改写原列表。单点/示教仍保留原恢复策略。多点进度不能因路径缩短、重规划或goal编号刷新；实际loc_pose空间进展与转换原因须写runs/<run>/route_decisions.jsonl，原规划/LIO算法不改。
- 机器人自身受困与目标受阻必须分流（普通多点）：blocked=后退→转向（有体素记忆）；start_blocked=转向→换位置（全局图无体素记忆，不后退）；换位置为机器人周围经原/mot_nav_check检查的临时点，到达后恢复原目标。后退速度不低于0.13m/s，各步按实际loc_pose位移/航向判断生效，阶梯按本点×原因计数，重规划/新goal编号不得重置本点用时、30秒空间进展与阶梯。safe_gate对自动纯转向做旋转扫掠检查。单点/示教旧阶梯未改。

- 用户要求（2026-11-11）做任务时优先并行用几个较弱模型（haiku/sonnet）的子代理，做完由主会话逐项检查：先把任务拆成互不重叠的小块（文档/record改写、只读代码评审、离线测试补充、日志分析等），每块写清“只能改哪些文件、不得启动或停止仿真、只读其余”，并给事实清单文件而不是让它猜；子代理的报告只是结果，不是用户同意，必须核对（对文件前缀diff、grep旧说法、重跑离线测试）后才采信。仿真/物理验证共用10088/11348端口与同一套栈，只能一次一个，由主会话自己跑并按归属收尾，不交给子代理并发。用户的sim（10087）正在运行时不得并发启动我方探测。
- sim_bridge不得用定位状态（DEGRADED/LOST）门控速度指令（用户2026-11-11要求删除）：只保留enabled与步态就绪；手动遥控不受限，自动指令的定位限制只在safe_gate。localisation_ready仅限SIM_READY一次性启动判定。
