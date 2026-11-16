> v4说明：本文件保留导入时的算法说明；本项目只使用 `/home/an/go2_v4/tools/sim_up.sh` 启动，不执行下文原工程的启动命令。当前接线和验证见根目录 record.md。

# 01 步态控制：启动、指令传递与反馈链条

包名 `go2_gait_controller`，ROS 节点名仍为 `go2_champ_controller`。当前默认可执行文件是 `reference_trot_controller`，实现文件 [reference_trot_controller.cpp](src/reference_trot_controller.cpp)。

它采用本地 `ROS2-Gazebo-GO2-main` 的对角相位和转向足端几何，再适配本项目的力矩接口、反馈与载荷交接。参考仓库：[ROS2-Gazebo-GO2](https://github.com/yanyuze1/ROS2-Gazebo-GO2)，原步态作者 mike4192、lnotspotl、abutalipovvv。CHAMP 在这里提供模型几何适配与 IK；当前没有调用 CHAMP 原来的步态生成器。

原闭环四拍慢走 [champ_controller.cpp](src/champ_controller.cpp)、[champ.yaml](config/champ.yaml) 完整保留；旧算法细节见 [README_legacy_crawl.md](README_legacy_crawl.md)。本文件下面的周期、参数和接口解释均针对默认新 trot，最后单独说明旧方案差异。

## 1. 使用入口

```bash
cd /home/an/internship
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch unitree_gazebo go2_world.launch.py enable_gait:=true
```

先自动站起，再由01接管。另一个终端：

```bash
source /home/an/internship/install/setup.bash
# 前进
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.14}, angular: {z: 0.0}}'
# 原地左转；先停止上一条发布进程
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 1.0}}'
# 原地右转
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: -1.0}}'
```

一次只运行一个速度发布器。停止原发布器后，显式零速度：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

参数入口 [reference_trot.yaml](config/reference_trot.yaml)：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| max_velocity_x | 0.14 | vx绝对上限，m/s |
| max_velocity_y | 0.04 | vy绝对上限，m/s |
| max_yaw_rate | 1.0 | yaw角速度绝对上限，rad/s |
| swing_height | 0.04 | 足端摆动高度，m |
| use_sim_time | true | 使用Gazebo仿真时间 |

当前相位时长、加速度限制和机身反馈增益写在 C++ 中，不是 YAML 可调项。`cfg.nominal_height=.33913` 用于初始化 CHAMP 几何；实际运行时的高度参考和名义足端从站稳后的测量记录，并非通过 `/body_pose` 设置。

## 2. 启动阶段：01何时获得控制权

### ① 总launch创建模型与节点

`unitree_gazebo/launch/go2_world.launch.py` 展开 xacro，启动物理暂停的 Gazebo、`robot_state_publisher` 和模型生成节点。`robot_state_publisher` 提供 `/robot_description` XML，Gazebo模型中的 `gazebo_ros2_control` 提供 controller_manager 和关节接口。

`enable_gait=true` 时同时启动本包的 `champ.launch.py`。名字仍是 champ.launch，但它默认选 `reference_trot_controller` 和 `reference_trot.yaml`，并不是默认运行旧 champ_controller。

### ② 控制器先加载，再由02激活站立链

模型生成成功后，总launch依次加载：

```text
joint_state_broadcaster（inactive）
    ↓
go2_joint_tracking_controller（inactive）
    ↓
go2_balance_effort_controller（inactive）
    ↓
02 stand_command.py
```

02请求激活 broadcaster 与站立 JTC，并恢复物理。它从实际关节角出发，给 `/go2_joint_tracking_controller/joint_trajectory` 发五秒站立轨迹，终点是 `[0,0.65,-1.3] × 4`。02随后退出，JTC仍保持终点。

02用steady clock轮询启动服务，避免暂停的仿真时间让启动流程停住。01的tick则使用仿真时间，恢复物理后才持续运行。

### ③ 01先收模型和反馈，不直接从低蹲开始走

构造函数创建订阅、发布和名义200 Hz（5 ms）定时器。

| 输入 | 回调保存或计算什么 |
|---|---|
| /robot_description | 只加载一次；RobotModel读取关节链、几何、轴、限位，Dynamics读取全身质量/惯量 |
| /joint_states | 按关节名称映射到FL/FR/RL/RR顺序，保存实际q、qd及收消息时间 |
| /go2/imu | 归一化四元数得到旋转矩阵R，保存角速度omega和时间 |
| /go2/FL/contact 等 | 保存接触力幅值均值和时间 |
| /cmd_vel | 检查有限性、限幅、保存requested_和最后指令时间 |

关节/IMU/四脚接触消息超过0.12 s则该周期不输出新目标。接管站姿条件：最大关节角误差≤0.12 rad，最大关节速度≤0.15 rad/s，并持续0.5 s仿真时间。

这里要求四脚接触消息新鲜，但新trot没有在初始化条件中额外检查四脚接触力全都超过阈值。消息新鲜和实际有接触是不同条件。

### ④ 从站姿建立名义参考

`dynamics_.evaluate(q_,R_)` 做实际关节FK，得到四脚在机身坐标中的位置。01记录：

- `nominal_`：站稳后的实际名义足端位置。
- `feet_`：足端轨迹的初始位置。
- `default_height_`：四脚经实测姿态旋转后的竖直距离平均值。
- `previous_desired_`：当前实测关节角，作为第一次角速度差分的参考。
- `desired_yaw_`：当前实测机身yaw。

高度参考是基于足端球心的几何量，不能直接当作Gazebo世界坐标中的绝对base高度。

### ⑤ 严格切换到行走执行层

01请求 `/controller_manager/switch_controller`：激活 `go2_balance_effort_controller`，停用 `go2_joint_tracking_controller`。响应成功后 `switched_=true`；切换后一秒内把运动目标置零，随后才接受非零速度。

站立与行走控制器不能同时争用effort接口。行走接管后，02不再参与；当前也没有旧讲解中的两秒关节角混合接管过程。

## 3. 运行时：一条前进指令如何传到关节

以 `linear.x=0.14, angular.z=0` 为例。

### ① /cmd_vel回调保存“想走多快”

回调读vx、vy、yaw_rate，分别截到配置上限，保存到 `requested_`。`linear.z` 和 `angular.x/y` 不用于当前运动目标。速度保持物理单位，没有参考仓库键盘输入的非线性缩放。

回调不生成轨迹、不算IK、不发布关节目标。外部10 Hz发消息，内部200 Hz持续计算；发送一条不是迈一步。

### ② tick检查反馈并平滑速度

`dt` 来自相邻仿真时间；`dt<=0` 或 `dt>0.1 s` 时跳过该周期。模型未加载、反馈过期或站姿初始化未完成时不进入正常运动计算。

指令超过0.5 s未更新、尚未接管、接管不足一秒或fault锁存时，目标速度置零。随后平滑到 `command_`：

```text
x/y加速度上限：0.15 m/s²
yaw加速度上限：0.5 rad/s²
```

从零加速到0.14 m/s至少约0.93 s；从零到1.0 rad/s至少约2 s，不含启动等待。

### ③ 固定相位决定哪一对腿摆动

```text
phase 0：四足支撑       0.04 s
phase 1：FL/RR摆动      0.18 s，FR/RL支撑
phase 2：四足支撑       0.04 s
phase 3：FR/RL摆动      0.18 s，FL/RR支撑
总周期：0.44 s
```

这是按仿真时间调度的对角trot，不会根据速度自动分类或切换walk/trot/gallop。接触测量用于支撑力分配与状态估计，没有用于等待实际落地后才推进相位。

### ④ 支撑与摆动产生四脚轨迹

支撑脚在机身水平朝向坐标中沿机身运动的反方向移动：

```cpp
feet = Rz(-turn * dt) * feet - [vx, vy, 0] * dt;
```

直行时脚相对身体往后移动，通过地面支撑推动身体向前。摆动脚的水平落点为：

```cpp
touchdown = Rz(turn * 0.11) * nominal + [vx, vy, 0] * 0.11;
```

`0.11=(0.04+0.18)/2` 是当前落点前伸系数，不能当作完整步长或实际速度保证。水平摆动从记录的摆动起点插值到落点，竖直位置为：

```text
z(u) = nominal_z + swing_height × 16u²(1−u)²，u∈[0,1]
```

u=0、1时回到名义高度，竖直速度为零；u=0.5达到抬脚峰值。

四足阶段用 `10u³−15u⁴+6u⁵` 平滑交接两组对角腿的支撑力上限，避免从一对支撑突然跳到另一对。

### ⑤ 实测姿态变换与CHAMP IK

轨迹坐标是按当前yaw定义的水平朝向坐标，不是世界坐标足端锚点。变换到机身坐标：

```cpp
target = R.transpose() * Rz(measured_yaw) * feet;
```

完整实测R包含roll/pitch，因此该变换会补偿机身倾斜。随后每条腿依次调用：

```cpp
champ::Kinematics::transformToHip(target, leg);
champ::Kinematics::inverse(hip, thigh, calf, leg, target);
```

输出十二个关节目标角，检查有限性和URDF限位。目标角速度由 `(本次目标−上次目标)/dt` 差分得到，截到±8 rad/s；这不是把角度目标本身按8 rad/s做限速。

不可达目标或绝对roll/pitch超过0.6 rad时锁存fault并保留上一批角度目标。该行为不是完整的翻倒恢复或安全落足控制。

## 4. 同时运行的机身反馈链

足端轨迹解决“迈哪里”，机身反馈解决“如何承重与纠偏”。它们在同一个tick内计算，共同构成发给03的目标。

### ① 从实际支撑脚估计速度和高度

Dynamics由实际q计算FK、全身CoM、组合惯量和各腿Jacobian。计划支撑且实际接触的脚参与估计；接触判定要求消息年龄<0.12 s、接触力幅值均值>1.5 N。

支撑脚无滑动时，机身平移速度近似为：

```text
v_base_world ≈ −R × (J(q) qd + omega × foot_body)
```

多支撑脚取平均，再用 `0.85×旧估计 + 0.15×新估计` 滤波。高度由支撑脚旋转后的竖直距离估计。没有Gazebo真实位置输入，也没有绝对位置保持；打滑或碰到墙体会破坏无滑动假设。

### ② 速度、高度与姿态误差得到机身需求

平滑速度按当前yaw转到世界坐标，与估计速度做差，生成目标水平加速度；高度误差和竖直速度生成高度修正。

目标yaw由 `desired_yaw_ += command_yaw_rate × dt` 积分。完整旋转矩阵的姿态误差在世界坐标中计算，再结合IMU角速度阻尼，得到目标角加速度及机身力矩。

不能直接把机身roll/pitch误差当作固定世界轴力矩：yaw较大时这会把修正施加到错误方向，这正是此前转向失稳的一处原因。

### ③ QP分配支撑反力

`go2_gait::distribute()` 依据足端相对CoM的力臂，将目标总力与力矩分到四脚：

- 只有计划支撑且实际接触的脚允许支撑力。
- 法向力非负，上限受四足阶段的载荷交接曲线调节。
- 水平力受摩擦锥近似约束。
- 对力变化做正则化，降低换支撑冲击。

这是约束反力分配，不是MPC；两足动态支撑也不具备旧四拍的三足静态稳定裕度保证。

### ④ 支撑力转成前馈关节力矩

对每条腿：

```text
τ_feedforward = −Jᵀ × Rᵀ × F_world + τ_gravity
```

Rᵀ将世界足端力转回机身坐标，Jᵀ将足端力转为关节力矩，重力项来自完整URDF模型。它随后与IK的角度/角速度一起发送。

## 5. 发布、03执行和Gazebo反馈

### ① 01发布命令消息

主输出 `/go2/balance_joint_command` 类型为 `sensor_msgs/msg/JointState`，在该话题中字段含义是：

| 字段 | 含义 |
|---|---|
| name | FL/FR/RL/RR各hip、thigh、calf，共12个关节 |
| position | q目标 |
| velocity | qd目标 |
| effort | 前馈力矩 |
| header.stamp | 当前仿真时间 |

它不是 `/joint_states` 的实测反馈，也不是 `JointTrajectory`。

### ② 03执行1000 Hz前馈＋PD

`03_joint_tracking_controller/src/balance_effort_controller.cpp` 校验关节名称顺序、数组长度和有限性，把目标写入实时缓冲区。

ros2_control以1000 Hz调用update，通过state interfaces直接读取实际q/qd，计算：

```text
τ = τ_feedforward + Kp(q_target−q_measured) + Kd(qd_target−qd_measured)
```

默认髋Kp=50、大腿/小腿Kp=100、Kd=2；无积分项。髋/大腿力矩截到±23.7 N·m，小腿截到±45.43 N·m。

目标超过0.1 s未更新时，03保持最后目标角度、目标角速度视为零并撤去前馈项，继续位置PD；不能把这种保持当作完整故障安全保证。

### ③ 仿真执行并返回两层反馈

03写入十二关节effort接口，`gazebo_ros2_control`施加力矩，Gazebo计算真实运动、碰撞、摩擦和重力。

下一轮反馈分两条：实际硬件关节状态返回03做PD；`/joint_states`、IMU和接触话题返回01做状态估计和机身纠偏。

```text
外部 /cmd_vel（10 Hz示例）
                 ↓
01回调：限幅、保存目标、记录指令时间
                 ↓
01 tick（200 Hz，仿真时间）
  ├─ 平滑速度→对角相位→足端轨迹→实测姿态变换→CHAMP IK→q/qd目标
  └─ 实际q/qd+IMU+接触→状态估计→机身力/力矩→QP→前馈力矩
                 ↓
/go2/balance_joint_command
                 ↓
03 go2_balance_effort_controller（1000 Hz）
  └─ 前馈+PD→力矩限幅→十二关节effort
                 ↓
gazebo_ros2_control→Gazebo物理运动
  ├─ 实际关节状态接口                     →03
  └─ /joint_states、/go2/imu、四脚/contact →01
```

02只负责启动站立；站立用JTC接管后inactive。01的IK、状态估计与QP都是编译在同一可执行文件中的库/函数，不是分别启动的ROS节点。

## 6. 原地转向和停止的具体行为

`vx=vy=0, yaw_rate=1` 时，目标yaw持续积分，`turn`还包含当前yaw误差修正：

```text
turn = clamp(command_yaw_rate + 2×wrap(desired_yaw−measured_yaw), −1.2, 1.2)
```

支撑脚反向旋转，摆动脚的落点向目标方向旋转，所以纯转向有明确的换脚落点；同时姿态反馈和QP产生机身转向力矩。另一对腿并不是等机身完全转完才抬起，而是两组按相位持续交替。

指令超时或显式零速度会先经过加速度限制减速。`command_`范数降到≤1e-4后，步态在phase 0的四足支撑窗口停下，四脚渐近回到名义位置。不会自动回到开始走的位置，也不会把yaw恢复到初始朝向；目标yaw保留停止时的积分结果。

默认trot不订阅 `/body_pose`。复制旧讲解中的抬高1 cm命令不会生效；旧四拍才有该接口，且它的控制流程不同。

## 7. 诊断接口与代码入口

腿顺序统一为FL、FR、RL、RR。IMU/接触传感器配置名义250 Hz，01名义200 Hz，03名义1000 Hz；均不能等同于一定达到对应墙钟频率，仿真可能运行较慢。

| 话题 | 类型与含义 |
|---|---|
| /go2/gait_contacts | UInt8MultiArray，计划支撑标签，不能冒充真实接触 |
| /go2/actual_contacts | UInt8MultiArray，实际接触判定 |
| /go2/foot_targets | PoseArray，base坐标下规划足端，不是实测脚位置 |
| /go2/balance_status | Float64MultiArray：phase、平滑vx、vy、yaw_rate、roll、pitch、yaw、fault |
| /go2/balance_joint_command | JointState，关节命令，见上文 |

`balance_status`中的速度是平滑后的指令，roll/pitch/yaw是IMU姿态；不能用该数组中的vx证明实际速度达标。新trot的fault需重启清除，零速度不会解除锁存。

| 文件 | 职责 |
|---|---|
| src/reference_trot_controller.cpp | 默认订阅、启动接管、轨迹、机身反馈、输出 |
| include/go2_gait_controller/planner.hpp | 保存CHAMP base与RobotModel适配对象 |
| include/go2_gait_controller/robot_model.hpp | 从XML读取Go2腿几何、关节轴、名称与限位 |
| include/go2_gait_controller/dynamics.hpp | 全身质量/惯量、FK/Jacobian、重力、约束支撑反力 |
| vendor/champ/kinematics/kinematics.h | 复用CHAMP IK |
| config/reference_trot.yaml | 默认trot输入限幅和步高 |
| launch/champ.launch.py | 选择可执行文件与配置，设置ROS节点名 |
| src/champ_controller.cpp / config/champ.yaml | 保留的旧四拍方案 |

## 8. 切回旧方案与旧讲解的区别

先停止当前仿真：

```bash
ros2 launch unitree_gazebo go2_world.launch.py enable_gait:=true gait_controller:=champ_controller gait_config:=/home/an/internship/src/go2_control/01_gait_controller/config/champ.yaml
```

| 项目 | 默认新trot | 保留的旧四拍 |
|---|---|---|
| 摆腿方式 | FL/RR与FR/RL成对交替 | FL→RR→FR→RL逐腿抬起 |
| 相位切换 | 按时间固定周期 | 移重心/卸载/离地/落地反馈条件 |
| 支撑判据 | 两足动态支撑、实际接触与反力分配 | 三足支撑三角形裕度 |
| vx默认限幅 | 0.14 m/s | 0.04 m/s |
| yaw_rate默认限幅 | 1.0 rad/s | 0.10 rad/s |
| /body_pose | 不订阅 | 支持高度及姿态偏置 |
| fault解除 | 重启 | 回到站立后零速度解除 |

两者的运行输出都使用 `/go2/balance_joint_command` 和独占effort执行层。更早的“100 Hz、Planner::calculate三连击、01持续向JTC发JointTrajectory、两秒混合接管”讲解不适用于这两个当前实现。

## 9. 验收与复现

最终110秒自由机身无障碍平地分段验收：0.04/0.14 m/s实测约0.0415/0.1322 m/s；正反向1.0 rad/s实测约±1.0006 rad/s。稳态直行倾角峰值<0.8°，转向<1.6°；转向后前进、显式停止和超时停止通过，无fault或翻倒。

存在约6%的高速直行速度误差与轻微漂移。测试无机身固定或运行中位置重置。独立Gazebo状态只用于验收，生产控制器不订阅它。当前未验收横移、混合转弯、障碍、坡地、推扰和真机；房间墙体碰撞要单独处理。

完整数据见 [diagnosis.md](../../diagnosis.md)，原始记录位于 `/home/an/internship/artifacts/trot_validation/`。

重新构建与数值回归：

```bash
cd /home/an/internship
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
colcon test --packages-select go2_gait_controller
colcon test-result --all
```

物理验收先完整退出其他仿真和速度发布器，再启动独立平地场景：

```bash
source /home/an/internship/install/setup.bash
ros2 launch unitree_gazebo go2_world.launch.py enable_gait:=true gui:=false world:=/home/an/internship/src/go2_control/01_gait_controller/test/trot_validation.world
```

另一个终端运行110秒测试和实际结果断言：

```bash
source /home/an/internship/install/setup.bash
/usr/bin/python3 /home/an/internship/src/go2_control/01_gait_controller/test/validate_trot.py /tmp/trot-results.json
/usr/bin/python3 /home/an/internship/src/go2_control/01_gait_controller/test/assert_trot_results.py /tmp/trot-results.json
```
