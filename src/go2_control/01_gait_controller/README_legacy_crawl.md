# Go2 ROS2 闭环四拍步态

包名 `go2_gait_controller`，节点名保留 `go2_champ_controller`；CHAMP 只复用几何适配与 IK，运行时不再使用原对角相位生成器或旧的开环 crawl。

## 启动与行走

```bash
cd /home/an/internship
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch unitree_gazebo go2_world.launch.py enable_gait:=true
```

02/原 JTC 先平滑站起；完整关节、IMU、四足接触及稳定站姿满足条件后，严格切换为独占的 `go2_balance_effort_controller`。

另开终端：

```bash
source /home/an/internship/install/setup.bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.04}, angular: {z: 0.0}}'
```

停止发布后 0.5 秒速度指令超时，完成当前落足再停止；也可显式发送零速度。

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

默认腿序为左前 FL→右后 RR→右前 FR→左后 RL。x 上限 0.04 m/s，输入 0.14 会截为 0.04；实际速度有误差。当前物理验收范围为平地低速前进、四腿独立抬脚、停止与重新起步，详见 `/home/an/internship/src/diagnosis.md`。

## 控制与文件

IMU、关节和实际足端接触→状态/全身 CoM 估计→支撑裕度与接触状态机→支撑力 QP / 摆动足端轨迹→实测姿态下 IK / Jacobian / 重力补偿→前馈+PD effort。

| 文件 | 职责 |
|---|---|
| src/champ_controller.cpp | 状态反馈、控制权切换、接触状态机、机身平衡与输出 |
| include/go2_gait_controller/dynamics.hpp | 完整 URDF 质量/惯量、FK、CoM、Jacobian、重力及约束反力分配 |
| include/go2_gait_controller/robot_model.hpp | CHAMP 的 Go2 几何、轴与关节限制适配 |
| 03_joint_tracking_controller/src/balance_effort_controller.cpp | 1000 Hz 前馈+PD、力矩限制、目标超时保持 |
| config/champ.yaml | 平衡步态时长、高度、裕度和输入限制 |
| test/test_kinematics.cpp | 实际 URDF 和反力约束回归检查 |

支撑脚锚点和摆动目标位于局部地面坐标，按测得机身位置和姿态转换。默认移重心参考时长 0.5 s、卸载渐变 0.25 s、摆动 0.4 s、摆动高度 4 cm；切换还必须满足实际反馈条件，因此不是固定周期。支撑规划裕度 2.5 cm，接触传感器约 250 Hz，步态层名义 200 Hz，effort 层 1000 Hz。

## 接口与故障恢复

| 话题 | 含义 |
|---|---|
| /cmd_vel | 目标机身 vx、vy、yaw_rate |
| /body_pose | 接管后机身位置/姿态偏置，z 相对接管高度，四元数必须有效 |
| /go2/imu | Gazebo IMU 的实际姿态/角速度 |
| /go2/FL/contact 等 | 四脚 Gazebo 实际接触数据 |
| /go2/actual_contacts | 接触测量判定 |
| /go2/gait_contacts | 计划支撑；不能冒充实际接触 |
| /go2/foot_targets | 局部地面坐标的规划足端目标 |
| /go2/balance_joint_command | 十二关节角度、速度和明确的前馈力矩目标 |
| /go2/balance_status | 状态诊断数组 |

`balance_status` 顺序：phase、leg、估计 base x/y/z、roll/pitch/yaw、支撑裕度、离地确认标记、FL/FR/RL/RR 接触力幅值均值；phase=0站立、1移重心、2卸载、3摆动、4落地、5恢复，leg 顺序 FL/FR/RL/RR。

失去支撑、裕度不足或不可达目标会锁存停止并尝试受控落足；回到站立后显式发送零速度解除锁存，再给新的行走指令。关节/IMU/接触过期会停止发送新目标，effort 控制器超时保留关节位置 PD，不能视为完整的故障安全保证。

`gait_config:=/absolute/path/config.yaml` 可替换参数；`single_leg_test` 取 0/1/2/3 时分别自动做一次 FL/FR/RL/RR 原地单腿试验，默认 -1 为行走。不要另启第二个步态节点或同时给原 JTC 发站立轨迹；默认不启用步态的站立方式仍保留。

修改后重新构建：

```bash
colcon build --base-paths src --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
ctest --test-dir build/go2_gait_controller --output-on-failure
```
