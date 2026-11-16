# v4 关节与步态执行层

导入自 internship，运行 reference_trot_controller。对角相位：FL/RR摆动、FR/RL支撑，随后交换，期间有四足交接阶段。

启动顺序由 go2_sim/sim.launch.py 统一管理；不要另起本目录控制器。输入速度由 /safe_cmd_vel 经 sim_bridge 门控到 /sim/cmd_vel，再重映射进入trot。运行期间同时保留IMU、关节、触地反馈与动态机身TF。

源算法的C++实现不做改动。真机 SDK 与 DDS 接口不在本项目内。
