# 仿真工具

| 工具 | 用途 |
|---|---|
| `sim_up.sh` | 先收尾上一轮v4，再启动新网页；网页按钮打开Gazebo |
| `sim_console.py` | 端口归属核实、旧网页停止、孤儿会话恢复、启动互斥与新网页就绪检查 |
| `check_sim.py --gui` | 临时端口 10088，完整检查导航、重启及三个停止入口 |
| `check_realtime.py --slam-exit-probe` | 自有10088会话，验证SLAM输入/唯一控制权、未知区阻挡、单点/前沿增图、客户端失联/手动/急停、保存及SLAM退出无真值回退 |
| `check_local_recovery.py` | 自有10088会话：用真实Gazebo箱体/墙制造受困（环境变量`GO2_LOCAL_SCENARIO`=`pocket`U形窄口｜`wallbox`墙+箱体（默认）｜`user`用户截图处的单箱体），同时跑多点（route）与单点（single_or_teach）两种策略，检查blocked=后退→转向、start_blocked=转向→换位置（不后退）、多点用时不被重规划重置（只验证逻辑与日志，不保证能脱困）。注意：地图外障碍可能使定位降级、sim_bridge冻结狗的运动，此时可能报“scenario did not produce blocked/start_blocked”，见docs/导航恢复现状.md“已知问题” |
| `check_recover_motion.py` | 自有10088会话：直接发RecoverCmd（后退/转向），记录/mot_recover_cmd_vel→/mot_cmd_vel→/safe_cmd_vel命令链、/safe_status与实际位移，用于诊断恢复步骤是否真的让狗动起来；`GO2_LOCAL_SCENARIO`=`open`空旷（默认）｜`wallbox`墙+箱体 |
| `check_teleop_degraded.py` | 自有10088会话：OK下遥控能动；生成大量地图外箱体使定位DEGRADED后，逐级记录teleop→mux→safe→/sim/cmd_vel与位移，定位丢弃发生在sim_bridge |
| `check_mode_sessions.py` | 同一个测试网页：固定图移动/重定位→停止→实时重启，检查初始定位反馈和旧TF/坐标不跨会话 |
| `check_route_view.py` | 自有10088会话：加载5点带朝向的任务并循环运行，保持到`$GO2_VIEW_DONE`文件出现（或`GO2_VIEW_MAX_S`，默认240秒），用来在浏览器里查看后续航段虚线和圈内朝向箭头；外层run_bounded至少`GO2_VIEW_MAX_S`+200秒 |
| `check_single_goal.py` | 自有10088会话：单点目标在地图外（goal_blocked起点型）应立即GOAL_REJECT并出现网页告警、不重试；邻近单点目标仍能到达 |
| `run_bounded.py <秒> <命令…>` | 构建和检查使用的超时、存活输出、独立进程组监测 |

板端、DDS 和真机诊断工具已从 v4 删除。请勿在这里加载 v3 或 internship 的 install 环境。


正常网页默认Leg-KILO，两个工作区共用仿真。检查示例：`source ./go2_env.sh`后执行`GO2_WATCH_STOP_SECONDS=85 python3 tools/run_bounded.py 480 python3 tools/check_realtime.py --slam-exit-probe`。不要与已运行仿真并发；先在归属网页停止。参考docs/架构.md与docs/操作.md的SLAM初始化和分层退出预算。
