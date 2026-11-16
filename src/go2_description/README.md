# Go2 仿真外观

URDF、DAE 与 mesh 资源导入自 `/home/an/internship/src/go2_description`。
移除旧 `radar` / `radar_joint`。Mid360 外观与传感器由 go2_sim 在启动时根据 geometry.yaml 加入，避免安装外参重复维护。
实际运行使用 xacro 与 ros2_control、站立和对角trot控制器。雷达安装参数在生成URDF时从geometry.yaml写入，随机身自然运动；不包含真机接口。
