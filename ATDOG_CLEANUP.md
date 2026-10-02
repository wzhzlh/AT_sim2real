# ATDog 工作区清理与构建

## 保留范围

- ATDog / ATDog2 / ATDog3：策略、模型、FSM、越障动作和实机入口。
- `rl_sar`：公共推理库、Gazebo 仿真及可选 MuJoCo 仿真。
- `obstacle_game`、`remote_node`、`robot_msgs`、`robot_joint_controller`。
- ATDog 电机、IMU、串口 SDK；现有 LibTorch；轨迹文件。

## 已完成

删除其他机器人模型、策略、实机入口、FSM、Unitree / Deeprobotics / Agibot / Zhinao SDK 和 LCM；删除 arm、arm_calc、arm_task、keyboard、launch_pack、obstacle_game_ui、task_game、顶层 robot_driver。

清除旧 build/install/log，删除对应 CMake 目标、SDK 初始化、无用优化器依赖及子模块声明。补齐 ATDog 与 ATDog3 的空 package.xml。构建入口不再检查 LCM，也不会用上游机器人库覆盖本地 ATDog 模型。

障碍赛启动文件已移除 UI 节点。`*_record.launch.py` 仅保留手动控制和遥控节点，不再提供原 UI 的录制功能。已有 trajectory 文件仍保留并可用于运行。

本轮仅清理项目内文件与依赖声明，没有卸载机器上的 apt/pip 包。

## 验证

2026-10-01：ROS2 Humble 下全新构建的 8 个包全部通过。验证产物位于 `/tmp/atdog-validation-build`、`/tmp/atdog-validation-install`，日志位于 `/tmp/atdog-validation-log`。非 ROS CMake 配置、package.xml 解析、Python launch 语法与构建脚本语法检查通过。

未启动实机。MuJoCo 库当前未安装，因此未验证 MuJoCo 编译或仿真运行。ONNX Runtime 当前未安装，ATDog 的 ONNX 策略需要安装该后端；ATDog2/3 的 Torch 策略仍需保留 LibTorch。构建通过不代表上述运行条件已满足。

## 使用

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

现有下载脚本可按需准备推理后端与 MuJoCo：

```bash
bash scripts/download_inference_runtime.sh onnx
bash scripts/download_mujoco.sh
```

## 剩余空间与限制

清理后源码约 69 MB、策略约 13 MB、推理运行时约 764 MB。另有约 722 MB 的 Git 历史，以及约 1.1 GB 正在使用的 `.vscode/browse.vc.db` 编辑器数据库。关闭编辑器后可清理 browse.vc.db 及同名 -shm/-wal/-lock 缓存；编辑器会重建索引。本轮没有删除正在使用的数据库。

Git 索引只读，删除和修改已反映在工作区，但没有暂存或提交；恢复可写环境后需将删除的子模块与 `.gitmodules` 一并暂存。未改写 Git 历史。
