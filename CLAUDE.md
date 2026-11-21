# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概览

这是一个机器人强化学习仿真与实机部署工作区，主要面向四足/轮式/人形机器人。仓库支持 ROS2 Foxy/Humble，以及独立 CMake 的 MuJoCo 仿真构建；实机部署使用 ROS2。

当前代码按 ROS 包组织在 `src/` 下，核心关系如下：

- `rl_sar`：上游强化学习控制框架核心包，提供 MuJoCo 仿真入口和不同机器人实机控制入口；可作为 ROS 包构建，也可通过顶层 `build.sh -m/-mj` 以纯 CMake 构建到 `cmake_build/bin`。
- `robot_msgs`：共享消息定义包，包含底层电机/机器人状态消息，以及本项目新增的 `Cmd`、`Remote`、`Int`、`BoxIdGrid` 等 ROS2 消息。
- `robot_joint_controller`：ROS2 控制器插件包，依赖 `robot_msgs`，保留通用关节控制器插件，`rl_sar` 不再依赖该包。
- `remote_node`：ROS2 遥控器串口驱动节点，读取串口协议并发布 `robot_msgs/msg/Remote` 到 `remote` 话题；如果系统没有 ROS `serial` 包，会回退编译 `src/rl_sar/library/thirdparty/robot_sdk/atdog/serial`。
- `obstacle_game`：障碍赛 ROS2 应用，`Robot` 订阅遥控器、读取 TF 位置、调用 `Pilot` 自动导航并发布 `robot_move_cmd`；`Record` 用于按遥控器按键记录路径 YAML。
- `task_game`：任务赛 ROS2 应用，结构与 `obstacle_game` 类似，但包含自定义行为树 `BT`。`Robot` 注册 `GeneratePlaneAction -> CatchBoxAction -> PlaceBoxAction` 顺序节点，通过黑板传递 `MoveBoxPlan` 和 `plan_index`。
- `rl_sar_zoo`：机器人描述资源包集合，用于 MuJoCo/URDF 等描述文件。

## 常用命令

### 准备依赖/子模块

```bash
git submodule update --init --recursive --recommend-shallow --progress
```

`build.sh` 会自动检查必要子模块 `src/rl_sar/library/thirdparty/lcm`，但首次克隆或依赖缺失时可以手动执行上面的命令。

### 构建

```bash
./build.sh
```

在当前 ROS 环境下构建全部 ROS 包：使用 `colcon build --merge-install --symlink-install`。

```bash
./build.sh task_game robot_msgs remote_node
```

只构建指定包。ROS2 下等价于 `colcon build --merge-install --symlink-install --packages-select ...`，适合开发单个包时使用。

```bash
./build.sh -m
./build.sh -mj
```

`-m/--cmake` 使用纯 CMake 构建 `src/rl_sar`，用于公共库构建，输出在 `cmake_build/bin` 和 `cmake_build/lib`；`-mj/--mujoco` 在纯 CMake 构建中启用 MuJoCo 支持。

```bash
./build.sh -c
./build.sh --clean task_game
```

清理构建产物和由脚本创建的 package.xml 符号链接。该命令会交互确认，并会删除 `build/`、`install/`、`log/`、`cmake_build/` 等构建目录。

### 直接使用 colcon（ROS2 开发常用）

```bash
colcon build --merge-install --symlink-install --packages-select task_game
source install/setup.bash
ros2 run task_game robot_control
```

`task_game` 和 `obstacle_game` 都安装 `robot_control`、`test_node` 两个可执行目标；`remote_node` 安装 `remote_node` 可执行目标。

```bash
colcon test --packages-select task_game
colcon test-result --verbose
```

运行单包测试/ament lint。当前业务包主要启用了 `ament_lint_auto`，`test_node` 只是普通可执行目标，不是 gtest/ament 测试用例。

### 运行仿真与节点

MuJoCo 构建后可直接运行：

```bash
./cmake_build/bin/rl_sim_mujoco <ROBOT> <SCENE>
```

例如 README 中的示例：`./cmake_build/bin/rl_sim_mujoco atdog2 scene`。

障碍赛已有 ROS2 launch 文件：

```bash
source install/setup.bash
ros2 launch obstacle_game atdog2_run.launch.py
ros2 launch obstacle_game remote_test.launch.py
```

`task_game` 当前没有 launch 文件，通常直接运行：

```bash
source install/setup.bash
ros2 run task_game robot_control
```

### 格式化/静态检查

仓库提供 `.clang-format`、`.clang-tidy` 和 `.pre-commit-config.yaml`。pre-commit 配置只包含 license header 插入 hook，并排除了 `src/rl_sar/library/core/matplotlibcpp` 和 `src/rl_sar/library/thirdparty`：

```bash
pre-commit run --all-files
```

ROS2 包的 CMake 在 `BUILD_TESTING` 时启用 `ament_lint_auto`，可用 `colcon test --packages-select <pkg>` 触发。

## 关键实现脉络

### 控制话题和模式切换

`remote_node` 发布 `robot_msgs/msg/Remote` 到 `remote`。`obstacle_game::Robot` 和 `task_game::Robot` 都订阅该话题，并发布 `robot_msgs/msg/Cmd` 到 `robot_move_cmd`。两者都通过遥控器按键在手动/自动控制之间切换：手动模式下根据摇杆填充 `cmd.vx/vy/vz`，自动模式下由 `Pilot` 根据 TF 中的 `odom -> base_link` 位姿生成控制命令。

### obstacle_game 架构

`obstacle_game` 的 `Robot` 是节点集成层：负责参数、TF、遥控器、控制定时器和命令发布。`Pilot` 负责根据 YAML 路径和当前状态生成自动导航命令；`Record` 在遥控器录制模式下把当前位置写入路径 YAML。`src/obstacle_game/README.md` 说明其自动导航思路：Robot 读取路程段信息，把当前策略号、上/下一个点位、速度/加速度限制和雷达位置交给 Pilot，Pilot 返回整狗控制命令。

### task_game 行为树架构

`task_game` 内置轻量行为树实现 `BT`，包含 `SequenceNode`、`FallbackNode`、`ConditionNode`、`ActionNode`，并通过 `std::any` 黑板传递节点间数据。`ActionNode` 执行后会请求下一轮从根节点重新开始，因此动作节点需要自行保存是否已完成等状态。

`task_game::Robot` 构造时注册根顺序节点：

1. `GeneratePlaneAction`：订阅/等待 `vip_box_id` 和 `box_id_grid`，生成 `std::vector<MoveBoxPlan>`，写入黑板键 `move_plan`，并初始化 `plan_index`。
2. `CatchBoxAction`：读取当前计划的抓取轨迹和源箱子位置，执行抓箱动作。
3. `PlaceBoxAction`：读取当前计划的放置轨迹、目标位置和 `place_at_second_floor`，执行放置并推进 `plan_index`。

`MoveBoxPlan` 定义在 `src/task_game/include/nodes/msg.hpp`，包含抓取轨迹、放置轨迹、源/目标箱子坐标和是否放第二层。

### 消息定义注意事项

`robot_msgs` 使用 ROS2 接口生成。修改消息后需要重新构建 `robot_msgs` 及依赖它的包，例如：

```bash
colcon build --merge-install --symlink-install --packages-select robot_msgs remote_node task_game obstacle_game
```

### 构建脚本行为

顶层 `build.sh` 使用 colcon 构建 ROS2 包，`-m` 构建公共库，`-mj` 构建 MuJoCo 仿真。包清单固定为 ROS2，无需生成版本切换链接。
