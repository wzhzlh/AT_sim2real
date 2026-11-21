# ATDog 工作区清理与构建

## 当前范围（2026-11-21）

保留 ATDog / ATDog2 / ATDog3 的 MuJoCo 仿真、ROS2 实机入口、状态机、全部动作及切换、URDF/MJCF/网格、现有策略与轨迹。保留公共推理库、TorchScript/ONNX、步态时钟、键盘与手柄、遥控、AprilTag 接口、动作数据加载、CSV、绘图、执行器网络及转换脚本、测试和第三方 SDK。

删除 Gazebo 控制程序 `rl_sim`、头文件、启动文件、world、专用控制配置、示例、下载脚本及 Docker 依赖；删除 ROS1 构建分支、启动文件与串口库旧打包接口；模型库仅保留 ATDog 系列和共享场景资源。构建脚本使用 ROS2 colcon，不再切换包清单。

ATDog 支持被动、起立、趴下、普通行走、楼梯、沙地和横杆状态；ATDog2/3 还保留坡地及桥面，ATDog2 保留关节检查姿态。FSM、策略文件与原机器人几何/动力学均未修改。

补齐三个地形场景原本缺少的 `unitree_hfield.png`（通用高度图，从原模型库提取）。模型包安装 URDF、网格、MJCF 及共享 AprilTag 场景文件。`obstacle_game` 的 launch 目录在本次清理前已被删除，安装规则现允许目录不存在。

## 构建与运行

ROS2 实机及辅助节点：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 run rl_sar rl_real_atdog2
# 其他入口：rl_real_atdog、rl_real_atdog3
```

独立 MuJoCo 仿真：

```bash
./build.sh -mj
./cmake_build/bin/rl_sim_mujoco atdog2 scene
# 机器人：atdog / atdog2 / atdog3
# 场景：scene / scene_terrain；atdog2/3 另有 scene_tag
```

启用 MuJoCo 的 ROS2 相机、速度与 AprilTag 桥接：

```bash
source /opt/ros/humble/setup.bash
cmake -S src/rl_sar -B cmake_build -DUSE_CMAKE=ON -DUSE_MUJOCO=ON -DMUJOCO_ROS2=ON
cmake --build cmake_build -j2
AT_MUJOCO_ROS2=1 ./cmake_build/bin/rl_sim_mujoco atdog2 scene_tag
```

`./build.sh -m` 仅构建公共库；实机入口需要 ROS2 构建。其余 AprilTag 使用说明见 `src/apriltag_localization/teach.md`。

## 验证与限制

- ROS2 Humble：9 个包编译和安装通过，包含三个实机入口。
- 独立 MuJoCo、带 ROS2 桥接的 MuJoCo 均编译通过；串口库独立构建通过。
- 8 个 ATDog MuJoCo 场景均经 MuJoCo 原生加载器加载通过；XML、URDF、Python、Shell 语法检查通过；现有步态时钟测试通过。
- 对保留文件进行 SHA256 对比：FSM、策略、原 MJCF、URDF 和网格保持一致；公共控制代码仅修改一处过时坐标系注释。
- 验证输出位于 `/tmp/atdog-cleanup-*`，没有启动实机、交互仿真窗口或重建现有工作区的 build/install。
- 原有策略缺失仍存在：ATDog 楼梯、沙地、横杆，以及 ATDog2 桥面配置引用的模型不在当前目录。本次保留原状，相关状态需补齐模型后使用。

只清理工作区源码与引用，没有卸载系统软件，也没有修改 Git 历史。第三方 SDK 的历史变更日志仍保留原文。
